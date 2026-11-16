package org.firstinspires.ftc.teamcode.logging;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.atomic.AtomicLong;

/** Bounded snapshots; deferred mode performs no disk I/O until a checkpoint or Stop. */
public final class AsyncCsvWriter2026 implements AutoCloseable {
    public interface Sink {
        boolean write(boolean detail, Object[] values);
    }

    private static final class Row {
        final boolean detail;
        final Object[] values;
        final long sequence;
        Row(boolean detail, Object[] values, long sequence) {
            this.detail = detail; this.values = values; this.sequence = sequence;
        }
    }

    private final File directory;
    private final String[] matchHeader, detailHeader;
    private final ArrayBlockingQueue<Row> queue;
    private final Thread worker;
    private final int reservedMatchSlots;
    private final boolean deferred;
    private final Object wakeup = new Object();
    private boolean workerStarted;
    private long acceptedSequence;
    private volatile long writeThrough, flushedThrough;
    private final AtomicLong droppedMatch = new AtomicLong(), droppedDetail = new AtomicLong();
    private volatile boolean accepting = true;
    private volatile String error = "";
    private volatile long written;

    public AsyncCsvWriter2026(File directory, String[] matchHeader, String[] detailHeader,
                              int capacity) {
        this(directory, matchHeader, detailHeader, capacity, false);
    }

    public AsyncCsvWriter2026(File directory, String[] matchHeader, String[] detailHeader,
                              int capacity, boolean deferred) {
        this.directory = directory;
        this.deferred = deferred;
        this.matchHeader = matchHeader.clone();
        this.detailHeader = detailHeader.clone();
        queue = new ArrayBlockingQueue<>(Math.max(16, capacity));
        reservedMatchSlots = Math.max(1, Math.max(16, capacity) / 8);
        worker = new Thread(this::run, "Pickles2026-log-writer");
        worker.setDaemon(true);
        worker.setPriority(Thread.MIN_PRIORITY);
        if (!deferred) startWorker();
    }

    /** Transfers ownership of a fresh values array. Never reuse/mutate it after this call. */
    public boolean write(boolean detail, Object[] values) {
        if (!accepting || !error.isEmpty()) return false;
        if (values.length != (detail ? detailHeader.length : matchHeader.length)) {
            error = "CSV schema mismatch";
            return false;
        }
        if ((detail && queue.remainingCapacity() <= reservedMatchSlots)
                || !queue.offer(new Row(detail, values, acceptedSequence + 1))) {
            (detail ? droppedDetail : droppedMatch).incrementAndGet();
            return false;
        }
        acceptedSequence++;
        if (!deferred && queue.size() == 1) signalWorker();
        return true;
    }

    /** Release only rows accepted so far. Later rows remain in memory until another checkpoint. */
    public void checkpoint() {
        if (!accepting || !error.isEmpty()) return;
        writeThrough = acceptedSequence;
        startWorker();
        signalWorker();
    }

    private synchronized void startWorker() {
        if (workerStarted) return;
        workerStarted = true;
        worker.start();
    }

    private void signalWorker() { synchronized (wakeup) { wakeup.notifyAll(); } }

    private void run() {
        BufferedWriter match = null, detail = null;
        Row inFlight = null;
        long processedThrough = 0;
        boolean dirty = false;
        boolean directoryReady = false;
        try {
            long lastFlush = System.nanoTime();
            while (accepting || !queue.isEmpty()) {
                Row head = queue.peek();
                if (head != null && (!deferred || !accepting || head.sequence <= writeThrough)) {
                    inFlight = queue.poll();
                    if (!directoryReady) {
                        if (!directory.isDirectory() && !directory.mkdirs() && !directory.isDirectory())
                            throw new IOException("Cannot create " + directory);
                        directoryReady = true;
                    }
                    if (inFlight.detail && detail == null) detail = open("shot_detail.csv", detailHeader);
                    if (!inFlight.detail && match == null) match = open("match.csv", matchHeader);
                    BufferedWriter destination = inFlight.detail ? detail : match;
                    destination.write(csv(inFlight.values));
                    destination.newLine();
                    written++;
                    processedThrough = inFlight.sequence;
                    dirty = true;
                    inFlight = null;
                }
                if (dirty && ((deferred && processedThrough >= writeThrough)
                        || System.nanoTime() - lastFlush >= 1_000_000_000L)) {
                    if (match != null) match.flush();
                    if (detail != null) detail.flush();
                    flushedThrough = processedThrough;
                    dirty = false;
                    lastFlush = System.nanoTime();
                }
                if (head == null || (deferred && accepting && head.sequence > writeThrough)) {
                    synchronized (wakeup) {
                        try { wakeup.wait(100); }
                        catch (InterruptedException ignored) { /* Recheck Stop/checkpoint. */ }
                    }
                }
            }
        } catch (IOException | RuntimeException exception) {
            error = exception.toString();
            accepting = false;
            if (inFlight != null) (inFlight.detail ? droppedDetail : droppedMatch).incrementAndGet();
            for (Row row; (row = queue.poll()) != null; )
                (row.detail ? droppedDetail : droppedMatch).incrementAndGet();
        } finally {
            closeWriter(match);
            closeWriter(detail);
            if (error.isEmpty()) flushedThrough = processedThrough;
        }
    }

    private BufferedWriter open(String name, String[] header) throws IOException {
        // java.nio.file is unavailable on older Control Hub Android versions.
        BufferedWriter result = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(new File(directory, name)), StandardCharsets.UTF_8));
        try {
            result.write(csv(header));
            result.newLine();
            return result;
        } catch (IOException exception) {
            closeWriter(result);
            throw exception;
        }
    }

    private void closeWriter(BufferedWriter writer) {
        if (writer == null) return;
        try { writer.close(); }
        catch (IOException exception) { error = exception.toString(); }
    }

    /** Stop accepting; the worker drains queued rows and closes files, without joining here. */
    @Override public void close() {
        accepting = false;
        writeThrough = acceptedSequence;
        startWorker();
        signalWorker();
    }
    public boolean awaitClosed(long milliseconds) throws InterruptedException {
        worker.join(Math.max(1, milliseconds));
        return !worker.isAlive();
    }
    public File getDirectory() { return directory; }
    public String getError() { return error; }
    public long getDroppedMatch() { return droppedMatch.get(); }
    public long getDroppedDetail() { return droppedDetail.get(); }
    public int getQueueSize() { return queue.size(); }
    public long getWritten() { return written; }
    public long getFlushedThrough() { return flushedThrough; }
    public synchronized boolean isWorkerStarted() { return workerStarted; }

    static String csv(Object[] values) {
        StringBuilder result = new StringBuilder(values.length * 12);
        for (int i = 0; i < values.length; i++) {
            if (i != 0) result.append(',');
            String value = values[i] == null ? "" : String.valueOf(values[i]);
            if (value.indexOf(',') >= 0 || value.indexOf('"') >= 0
                    || value.indexOf('\n') >= 0 || value.indexOf('\r') >= 0)
                result.append('"').append(value.replace("\"", "\"\"")).append('"');
            else result.append(value);
        }
        return result.toString();
    }
}
