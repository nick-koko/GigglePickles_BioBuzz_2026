import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.nio.file.Files;
import java.util.*;
import org.firstinspires.ftc.teamcode.logging.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.shot.ShotCalibrationTable2026;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Desktop checks against production logging code; no Android Init or robot hardware. */
public final class Logging2026Test {
    private static int checks;
    public static void main(String[] args) throws Exception {
        schemaAndDetail();
        sequenceStartOccupancy();
        eventsAndPause();
        shooterProfileEvents();
        detailSwitchAvoidsMotorReads();
        configurationIncludesCalibration();
        deferredWrites();
        checkpointPolicy();
        writerAndErrors();
        System.out.println("Passed " + checks + " logging checks.");
    }

    private static void schemaAndDetail() throws Exception {
        check("match schema width", new RobotLogFrame2026().values("SAMPLE", "").length == RobotLogFrame2026.HEADER.length);
        check("unique match columns", new HashSet<>(Arrays.asList(RobotLogFrame2026.HEADER)).size() == RobotLogFrame2026.HEADER.length);
        check("unique detail columns", new HashSet<>(Arrays.asList(RobotLogging2026.DETAIL_HEADER)).size() == RobotLogging2026.DETAIL_HEADER.length);
        check("memory storage is default", RobotLogging2026.STORAGE_MODE == RobotLogging2026.StorageMode.MEMORY_UNTIL_STOP);
        check("practice-only flags default off", !RobotLogging2026.PRACTICE_MODE && !RobotLogging2026.ENABLE_POST_SHOT_PRACTICE_WRITES);
        ShotSequenceController2026 shots = new ShotSequenceController2026();
        RobotLogging2026 logging = new RobotLogging2026();
        logging.start("Test", "TUNING", shots, false);
        Method details = RobotLogging2026.class.getDeclaredMethod("detailValues", long.class);
        details.setAccessible(true);
        Object[] values = (Object[])details.invoke(logging, System.nanoTime());
        check("detail schema matches actual capture", values.length == RobotLogging2026.DETAIL_EXTRA_HEADER.length);
        Map<String,Object> row = new HashMap<>();
        for (int i=0;i<values.length;i++) row.put(RobotLogging2026.DETAIL_EXTRA_HEADER[i],values[i]);
        check("PID field aligned", row.get("kP").equals(ShooterSubsystem2026.kP));
        check("voltage field aligned", row.get("voltage_comp_max_gain").equals(ShooterSubsystem2026.VOLTAGE_COMP_MAX_GAIN));
        check("boost config aligned", row.get("boost_enabled_for_sequence").equals(false));
        check("cached controller target aligned", row.get("controller_target_rpm")
                .equals(ShooterSubsystem2026.INSTANCE.getControllerTargetFlywheelRpm()));
        check("startup flag aligned", row.get("startup_profile_enabled").equals(false));
        check("asymmetric flag aligned", row.get("asymmetric_slew_enabled").equals(false));
        check("active up limit aligned", row.get("active_slew_up_per_s").equals(ShooterSubsystem2026.INSTANCE.getActiveSlewUpPerSecond()));
        check("active down limit aligned", row.get("active_slew_down_per_s").equals(ShooterSubsystem2026.INSTANCE.getActiveSlewDownPerSecond()));
        check("controller-sample override aligned", row.get("controller_sample_override_floor").equals(ShooterSubsystem2026.INSTANCE.getSampleOverrideFloor()));
        check("boost voltage gain aligned", row.get("boost_voltage_gain").equals(1.0));
        check("boost taper multiplier aligned", row.get("boost_overspeed_multiplier").equals(1.0));
        check("boost effective floor aligned", row.get("boost_floor_effective").equals(0.0));
        check("no uninitialized turret angle invented", Double.isNaN((Double)row.get("turret_commanded_deg")));
        check("no nonexistent hood columns", Arrays.stream(RobotLogging2026.DETAIL_HEADER).noneMatch(s -> s.contains("hood")));
        logging.stop();
    }

    private static void sequenceStartOccupancy() throws Exception {
        IntakeWithSensorsSubsystem2026 intake = IntakeWithSensorsSubsystem2026.INSTANCE;
        intake.setBallCount(4);
        Field beams = intake.getClass().getDeclaredField("cachedBeamMask");
        beams.setAccessible(true); beams.setInt(intake, 15);
        ShotSequenceController2026 shots = new ShotSequenceController2026();
        shots.reset(); shots.start();
        check("start ball count preserved", shots.getStartBallCount()==4);
        check("start raw beams preserved", shots.getStartBeamMask()==15);
        shots.update();
        check("normal feed still clears tracked count", intake.getBallCount()==0);
        check("start count survives feed clear", shots.getStartBallCount()==4);
        Field start = shots.getClass().getDeclaredField("sequenceStartNanos");
        start.setAccessible(true); start.setLong(shots, System.nanoTime()-2_000_000_000L);
        shots.update();
        check("normal timer completion observed", !shots.isActive() && shots.getEndReason().equals("FEED_TIMER_COMPLETE"));
        shots.cancel();
        check("idle cancellation does not overwrite completion", shots.getEndReason().equals("FEED_TIMER_COMPLETE"));
        shots.start(); shots.cancel();
        check("active cancellation observed", shots.getEndReason().equals("CANCELLED"));
    }

    private static RobotLogFrame2026 frame(double ms, int id, boolean active, int beams) {
        RobotLogFrame2026 f = new RobotLogFrame2026();
        f.timeMs=ms; f.sequenceId=id; f.sequenceActive=active; f.beams=beams;
        return f;
    }
    private static void eventsAndPause() {
        List<Object[]> rows = new ArrayList<>();
        MatchLogRecorder2026 recorder = new MatchLogRecorder2026((detail, values) -> {rows.add(values);return true;});
        recorder.update(frame(0,0,false,0),true,50);
        recorder.update(frame(1,1,true,1),true,50);
        check("shot start between periodic samples retained", count(rows,"SHOT_START")==1);
        check("beam edge between periodic samples retained", count(rows,"BEAM_EDGE")==1);
        recorder.update(frame(2,1,false,0),true,50);
        check("sequence end retained", count(rows,"SHOT_END")==1);
        check("no invented scored ball event", count(rows,"SCORE")==0);
        int before=rows.size();
        recorder.update(frame(100,2,true,15),false,50);
        recorder.update(frame(110,2,false,0),false,50);
        check("disabled basic stream writes nothing", before==rows.size());
        recorder.update(frame(150,2,false,0),true,50);
        check("resume event retained", count(rows,"LOG_RESUME")==2);
        recorder.update(frame(151,3,true,0),true,50);
        check("IDs continue after pause", count(rows,"SHOT_START")==2);
        MatchLogRecorder2026 firstActive = new MatchLogRecorder2026((d,v)->{rows.add(v);return true;});
        firstActive.update(frame(0,1,true,0),true,50);
        check("first-loop feed captured", count(rows,"SHOT_START")==3);
    }
    private static long count(List<Object[]> rows, String event) {
        int index = Arrays.asList(RobotLogFrame2026.HEADER).indexOf("event");
        return rows.stream().filter(r->event.equals(r[index])).count();
    }

    private static void shooterProfileEvents() {
        List<Object[]> rows=new ArrayList<>();
        MatchLogRecorder2026 recorder=new MatchLogRecorder2026((d,v)->{rows.add(v);return true;});
        RobotLogFrame2026 first=frame(0,0,false,0);
        recorder.update(first,true,1000);
        RobotLogFrame2026 startup=frame(1,0,false,0); startup.shooterPowerProfile="STARTUP";
        recorder.update(startup,true,1000);
        RobotLogFrame2026 handoff=frame(2,0,false,0); handoff.shooterPowerProfile="RUNNING";
        handoff.shooterStartupExitReason="SETTLED"; handoff.shooterStartupElapsed=.8;
        recorder.update(handoff,true,1000);
        check("startup and handoff events between samples",count(rows,"SHOOTER_PROFILE")==2);
        Object[] last=rows.get(rows.size()-1);
        check("profile state stored in basic rows",last[Arrays.asList(RobotLogFrame2026.HEADER).indexOf("shooter_power_profile")].equals("RUNNING"));
        check("profile reason stored in basic rows",last[Arrays.asList(RobotLogFrame2026.HEADER).indexOf("shooter_startup_exit_reason")].equals("SETTLED"));
    }

    private static void detailSwitchAvoidsMotorReads() throws Exception {
        IntakeWithSensorsSubsystem2026 intake = IntakeWithSensorsSubsystem2026.INSTANCE;
        Field intakeField=intake.getClass().getDeclaredField("intakeMotor");
        Field transferField=intake.getClass().getDeclaredField("transferMotor");
        intakeField.setAccessible(true); transferField.setAccessible(true);
        Object originalIntake=intakeField.get(intake), originalTransfer=transferField.get(intake);
        boolean match=RobotLogging2026.ENABLE_MATCH_LOG, detail=RobotLogging2026.ENABLE_SHOT_DETAIL_LOG;
        int[] velocityReads={0}, powerReads={0};
        DcMotorEx motor=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class},(proxy,method,arguments)->{
                    if(method.getName().equals("getVelocity")) {velocityReads[0]++;return 1200.0;}
                    if(method.getName().equals("getPower")) {powerReads[0]++;return .5;}
                    throw new AssertionError("Unexpected hardware access: "+method.getName());
                });
        AsyncCsvWriter2026 writer=new AsyncCsvWriter2026(Files.createTempDirectory("pickles-live-flags-").toFile(),
                RobotLogFrame2026.HEADER,RobotLogging2026.DETAIL_HEADER,64,true);
        RobotLogging2026 logging=new RobotLogging2026();
        try {
            intakeField.set(intake,motor); transferField.set(intake,motor);
            logging.start("Test","TUNING",new ShotSequenceController2026(),false);
            Field writerField=RobotLogging2026.class.getDeclaredField("writer");
            Field recorderField=RobotLogging2026.class.getDeclaredField("recorder");
            writerField.setAccessible(true); recorderField.setAccessible(true);
            writerField.set(logging,writer); recorderField.set(logging,new MatchLogRecorder2026(writer::write));
            RobotLogging2026.ENABLE_MATCH_LOG=true;
            RobotLogging2026.ENABLE_SHOT_DETAIL_LOG=false;
            logging.update(null,null,"TEST","BENCH","MANUAL");
            check("basic capture adds no motor reads",velocityReads[0]==0 && powerReads[0]==0);
            RobotLogging2026.ENABLE_SHOT_DETAIL_LOG=true;
            logging.update(null,null,"TEST","BENCH","MANUAL");
            check("detail samples each motor velocity once",velocityReads[0]==2);
            check("detail samples each motor power readback once",powerReads[0]==2);
            RobotLogging2026.ENABLE_MATCH_LOG=false;
            Field nextConfig=RobotLogging2026.class.getDeclaredField("nextConfigurationNanos");
            nextConfig.setAccessible(true); nextConfig.setLong(logging,0);
            logging.update(null,null,"DETAIL_ONLY","BENCH","MANUAL");
            check("detail-only capture samples each motor once",velocityReads[0]==4 && powerReads[0]==4);
            RobotLogging2026.ENABLE_SHOT_DETAIL_LOG=false;
            RobotLogging2026.ENABLE_MATCH_LOG=false;
            logging.update(null,null,"TEST","BENCH","MANUAL");
            check("live disable stops extra reads",velocityReads[0]==4 && powerReads[0]==4);
            check("detail enabled does not start disk writer", !writer.isWorkerStarted());
            logging.stop();
            check("stop drains with both flags disabled",writer.awaitClosed(5000));
            check("live flag changes preserve captured detail",new File(writer.getDirectory(),"shot_detail.csv").isFile());
            check("live flag test has no writer errors",writer.getError().isEmpty());
            String detailText=new String(Files.readAllBytes(new File(writer.getDirectory(),"shot_detail.csv").toPath()),
                    java.nio.charset.StandardCharsets.UTF_8);
            check("detailed-only configuration retained",detailText.contains("RobotLogging2026.ENABLE_MATCH_LOG=false"));
            check("detail includes build provenance",detailText.contains("RobotLogging2026.BUILD_LABEL="));
            check("detail configuration metadata has no fabricated measurements",
                    detailText.contains(",CONFIG,") && detailText.contains(",DETAIL,"));
        } finally {
            logging.stop(); writer.close();
            intakeField.set(intake,originalIntake); transferField.set(intake,originalTransfer);
            RobotLogging2026.ENABLE_MATCH_LOG=match; RobotLogging2026.ENABLE_SHOT_DETAIL_LOG=detail;
        }
    }

    private static void configurationIncludesCalibration() throws Exception {
        ShotCalibrationTable2026.Sample[] original=ShotCalibrationTable2026.SAMPLES;
        try {
            ShotCalibrationTable2026.SAMPLES=new ShotCalibrationTable2026.Sample[]{
                    new ShotCalibrationTable2026.Sample(10,20,3100,1,-2),null};
            RobotLogging2026 logging=new RobotLogging2026();
            logging.start("Test","TUNING",new ShotSequenceController2026(),false);
            Method method=RobotLogging2026.class.getDeclaredMethod("configuration");
            method.setAccessible(true);
            String text=(String)method.invoke(logging);
            check("table interpolation settings retained",text.contains("ShotCalibrationTable2026.NEAREST_SAMPLE_COUNT="));
            check("table sample values retained",text.contains("SAMPLES[0]=10.0,20.0,3100.0,1.0,-2.0"));
            check("null table samples tolerated",text.contains("SAMPLES[1]=null"));
            check("new startup defaults in config snapshot",text.contains("ShooterSubsystem2026.ENABLE_STARTUP_PROFILE=false"));
            check("new boost defaults in config snapshot",text.contains("ShotSequenceController2026.ENABLE_RAMPED_BOOST=false"));
            logging.stop();
        } finally { ShotCalibrationTable2026.SAMPLES=original; }
    }

    private static void deferredWrites() throws Exception {
        File directory=new File(Files.createTempDirectory("pickles-deferred-").toFile(),"session");
        AsyncCsvWriter2026 writer=new AsyncCsvWriter2026(directory,new String[]{"n"},new String[]{"n"},32,true);
        writer.write(false,new Object[]{1}); writer.write(true,new Object[]{2});
        check("deferred capture creates no directory",!directory.exists());
        check("deferred capture starts no worker",!writer.isWorkerStarted());
        check("deferred capture retains all rows",writer.getQueueSize()==2 && writer.getWritten()==0);
        writer.checkpoint();
        writer.write(false,new Object[]{3});
        long deadline=System.nanoTime()+5_000_000_000L;
        while(writer.getFlushedThrough()<2 && System.nanoTime()<deadline) Thread.sleep(2);
        check("checkpoint completes",writer.getFlushedThrough()==2);
        check("checkpoint leaves later rows in memory",writer.getWritten()==2 && writer.getQueueSize()==1);
        check("checkpoint creates and flushes both files",Files.readAllLines(new File(directory,"match.csv").toPath()).size()==2
                && Files.readAllLines(new File(directory,"shot_detail.csv").toPath()).size()==2);
        writer.checkpoint();
        writer.write(true,new Object[]{4});
        writer.close();
        check("Stop drains remaining rows",writer.awaitClosed(5000) && writer.getWritten()==4);
        check("checkpoints append without overwriting",Files.readAllLines(new File(directory,"match.csv").toPath()).equals(Arrays.asList("n","1","3")));
        check("Stop flushes final detail row",Files.readAllLines(new File(directory,"shot_detail.csv").toPath()).equals(Arrays.asList("n","2","4")));
        File stoppedDirectory=new File(directory,"stop-only");
        AsyncCsvWriter2026 stopped=new AsyncCsvWriter2026(stoppedDirectory,new String[]{"n"},new String[]{"n"},16,true);
        for(int i=0;i<100;i++) stopped.write(true,new Object[]{i});
        check("memory full never starts disk writes",!stopped.isWorkerStarted() && !stoppedDirectory.exists());
        check("memory capacity remains bounded",stopped.getQueueSize()<=16 && stopped.getDroppedDetail()>0);
        check("basic capacity reserved",stopped.write(false,new Object[]{101}));
        long pending=stopped.getQueueSize();
        stopped.close();
        check("buffer-full Stop saves retained rows",stopped.awaitClosed(5000) && stopped.getWritten()==pending);
    }

    private static void checkpointPolicy() {
        LogWritePolicy2026 policy=new LogWritePolicy2026();
        double autoWindow=RobotLogging2026.AUTO_MEMORY_ONLY_SECONDS;
        double teleopWindow=RobotLogging2026.TELEOP_MEMORY_ONLY_SECONDS;
        check("auto default includes timing margin",autoWindow==35.0);
        check("teleop default includes timing margin",teleopWindow==125.0);
        check("auto window includes exact 35 seconds",policy.update(35,false,autoWindow,true,10,false,false,true,1).isEmpty());
        check("auto long-practice checkpoint starts beyond 35",policy.update(35.001,false,autoWindow,true,10,false,false,true,1).equals("LONG_PRACTICE"));
        check("practice checkpoint not every loop",policy.update(35.01,false,autoWindow,true,10,false,false,true,1).isEmpty());
        check("practice checkpoints repeat at interval",policy.update(45.01,false,autoWindow,true,10,false,false,true,1).equals("LONG_PRACTICE"));
        policy=new LogWritePolicy2026();
        check("teleop window includes exact 125 seconds",policy.update(125,false,teleopWindow,true,10,false,false,true,1).isEmpty());
        check("teleop long-practice checkpoint beyond 125",policy.update(125.001,false,teleopWindow,true,10,false,false,true,1).equals("LONG_PRACTICE"));
        policy=new LogWritePolicy2026();
        check("long-practice checkpoints can be disabled",policy.update(1000,false,30,false,10,false,false,true,1).isEmpty());
        policy.update(1,true,30,true,10,false,false,true,1);
        policy.update(2,false,30,true,10,false,false,true,1);
        check("detail alone never writes after a shot",policy.update(3,false,30,true,10,false,false,true,1).isEmpty());
        check("post-shot switch still requires practice mode",policy.update(3.1,false,30,true,10,false,true,true,1).isEmpty());
        check("practice mode alone cannot enable early writing",policy.update(3.2,false,30,true,10,true,false,true,1).isEmpty());
        check("post-shot checkpoint requires detailed capture",policy.update(3.3,false,30,true,10,true,true,false,1).isEmpty());
        check("explicit practice post-shot checkpoint",policy.update(3.4,false,30,true,10,true,true,true,1).equals("POST_SHOT_PRACTICE"));
        check("post-shot checkpoint occurs once",policy.update(3.5,false,30,true,10,true,true,true,1).isEmpty());
        policy.update(4,true,30,true,10,true,true,true,1);
        policy.update(5,false,30,true,10,true,true,true,1);
        check("post-shot delay keeps recovery tail in memory",policy.update(5.5,false,30,true,10,true,true,true,1).isEmpty());
        check("a new active shot prevents post-shot checkpoint",policy.update(6,true,30,true,10,true,true,true,1).isEmpty());
    }

    private static void writerAndErrors() throws Exception {
        File directory = Files.createTempDirectory("pickles-log-test-").toFile();
        AsyncCsvWriter2026 writer = new AsyncCsvWriter2026(directory, new String[]{"n","text"},new String[]{"n","text"},16);
        check("initial row accepted", writer.write(false,new Object[]{1,"comma, quote\" and\nnewline"}));
        long accepted=1;
        for(int i=0;i<20000;i++) if(writer.write(true,new Object[]{i,"diagnostic"})) accepted++;
        check("bounded queue",writer.getQueueSize()<=16);
        check("overload visible",writer.getDroppedDetail()>0);
        writer.close();
        check("worker drains on stop",writer.awaitClosed(5000));
        check("all accepted rows written",writer.getWritten()==accepted);
        check("all offered rows accounted",accepted+writer.getDroppedDetail()==20001);
        check("CSV special characters escaped",Files.readString(new File(directory,"match.csv").toPath()).contains("\"comma, quote\"\" and\nnewline\""));
        check("no write after stop",!writer.write(false,new Object[]{1,"late"}));
        check("normal writer no error",writer.getError().isEmpty());
        File obstruction=File.createTempFile("pickles-logging-obstruction-",".tmp");
        AsyncCsvWriter2026 failing = new AsyncCsvWriter2026(obstruction,new String[]{"x"},new String[]{"x"},16);
        failing.write(false,new Object[]{1}); failing.close();
        check("I/O failure terminates worker",failing.awaitClosed(5000));
        check("I/O failure exposed without throwing on producer",!failing.getError().isEmpty());
        AsyncCsvWriter2026 mismatch=new AsyncCsvWriter2026(new File(directory,"schema"),new String[]{"x"},new String[]{"x"},16);
        check("bad schema rejected",!mismatch.write(true,new Object[]{1,2}));
        mismatch.close(); check("schema failure drains",mismatch.awaitClosed(5000));
        check("schema error exposed",!mismatch.getError().isEmpty());
    }
    private static void check(String label,boolean condition) {
        checks++; if(!condition) throw new AssertionError(label);
    }
}
