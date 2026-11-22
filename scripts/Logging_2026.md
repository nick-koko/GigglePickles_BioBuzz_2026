# Pickles 2026 match and shot logging

Logging is integrated into `Pickles2026Teleop`, `CloseShootingAlone`, `CloseWithPartner`,
`FarWithPartner`, and `ShooterTuning2026` (which retains its existing Disabled annotation).
Implementation lives in `teamcode/logging`, with small lifecycle calls in each OpMode.
There is no hood data and no dependency on the 2025 controllers or WPILib on the robot.

**Shot Recommended 2026**, **Flywheel Model 2026**, **Shot Compare 2026** and
**Shot Repeatability 2026** are stationary test autonomous programs documented in
[Shot_Experiments_2026.md](Shot_Experiments_2026.md). They add typed trial/profile/
condition/phase metadata, a frozen `TEST_PLAN`, and feed video cues to these same logs.
The unloaded model program records `test_feed_enabled=false` and a confirmed load
of zero, with no automatic feed/shot event.
Intake/transfer shot-feed powers, velocity/power commands and measured motor RPMs
are included; the optional feeding matrix changes only scoped test values.
Normal opmode defaults are restored on completion, Stop and exceptional cleanup.

Shooter power/startup/boost configuration is documented in
[Shooter_Power_2026_Setup.md](Shooter_Power_2026_Setup.md). Its new behavior defaults
off; existing slew, voltage compensation, boost floor and timing are preserved.
Basic CSVs now append shooter profile state, elapsed startup time and handoff reason,
with `SHOOTER_PROFILE` events. Detailed CSVs append active/configured slew limits,
RPM slope, snapshotted boost profile parameters, and unscaled/scaled/tapered floors.
Pending shot requests and cached controller-sample floors are separate fields.
Existing column names and schema version 1 are retained; the converter reads both
old and extended logs. These additions reuse cached samples without new hardware reads.

Intake recovery adds `intake_capacity_override` and `ball_count_recovery_events`
to basic rows, with `INTAKE_OVERRIDE` and `BALL_COUNT_RECOVERY` events. Detail rows
also record the held override request, capacity reached, sustained-recovery settings,
confirmed stack, last recovery counts/mask and each beam's confirmation timer.
These are collection-eligible timers, reset during feed/reverse/diagnostic motion,
not a claim of physical beam-clear events. See [Intake_LED_2026_Setup.md](Intake_LED_2026_Setup.md).

## Controls in Panels

Open **RobotLogging2026**. Settings are shared by these OpModes.

| Setting | Default | Meaning |
| --- | --- | --- |
| `ENABLE_MATCH_LOG` | `true` | Basic replay samples and events |
| `ENABLE_SHOT_DETAIL_LOG` | `true` | Independent detailed CSV |
| `MATCH_SAMPLE_INTERVAL_MS` | `50` | Basic pose samples at approximately 20 Hz |
| `DETAIL_SAMPLE_INTERVAL_MS` | `0` | Every observed OpMode loop; increase to downsample |
| `STORAGE_MODE` | `MEMORY_UNTIL_STOP` | Buffer snapshots without disk activity during normal match windows; choose before Play |
| `MEMORY_MAX_ROWS` | `40000` | Bounded pending snapshots in memory mode |
| `WRITER_QUEUE_CAPACITY` | `2048` | Pending rows for optional `BACKGROUND_STREAMING` mode |
| `AUTO_MEMORY_ONLY_SECONDS` | `35` | Autonomous window plus timing margin without automatic disk writes |
| `TELEOP_MEMORY_ONLY_SECONDS` | `125` | Teleop window plus timing margin; also used by shooter tuning |
| `ENABLE_LONG_PRACTICE_WRITES` | `true` | Allow periodic checkpoints strictly beyond that window |
| `PRACTICE_CHECKPOINT_INTERVAL_SECONDS` | `10` | Interval between long-practice checkpoints |
| `PRACTICE_MODE` | `false` | Explicitly designate this run as practice for early post-shot writes |
| `ENABLE_POST_SHOT_PRACTICE_WRITES` | `false` | Also required, along with detailed logging, for early post-shot writes |
| `POST_SHOT_WRITE_DELAY_SECONDS` | `1` | Wait after a sequence ends to capture its recovery tail before a practice checkpoint |
| `PROFILE_LABEL` | `baseline` | Tag every row, e.g. `added_mass_no_boost` |
| `RUN_NOTES` | empty | Configuration notes in either stream, e.g. battery, flywheel mass, preload technique |
| `BUILD_LABEL` | `unspecified` | Manually entered source revision/build tag, included in configuration snapshots |

Basic events are checked every loop regardless of the basic sample interval. Detail capture
continues throughout the run, including pickup, flywheel spin-up and recovery, so transitions
before/after feeding are available. Flags and sample intervals can change live; turning a
stream off does not discard already recorded rows. Stop still drains/flushes queued rows.
If both flags are off at Play, the memory buffer is created lazily if a stream is enabled later.
Logging starts at Play, so there are no logging motor reads or actuator commands during Init.

### Default storage behavior

In `MEMORY_UNTIL_STOP`, snapshots stay in bounded memory. No disk-writer thread starts and
no directories/files are created until Stop or an explicitly permitted practice checkpoint.
Autonomous remains memory-only through 35.0 seconds since Play, and teleop through 125.0 seconds,
even when detailed logging is enabled. Stop releases the buffer to a background saver so motor
shutdown does not wait for disk. Let saving finish before downloading or shutting down the hub.

Runs longer than those windows are treated as extended practice: the first checkpoint occurs
strictly after the window, then at most once every 10 seconds by default. Each checkpoint
releases only the snapshots already buffered; new snapshots remain in memory for the next
checkpoint or Stop. Files are appended, so there is no duplicate export or overwriting.
Set `ENABLE_LONG_PRACTICE_WRITES=false` to keep the entire run in memory until Stop.

For optional post-shot saves during tuning, explicitly enable **both** `PRACTICE_MODE` and
`ENABLE_POST_SHOT_PRACTICE_WRITES`, with detailed logging on. A checkpoint is allowed after
the feed sequence ends and the recovery delay passes. A new active sequence before that delay
cancels the pending checkpoint. A writer that has already started may continue into a later
sequence; this option knowingly permits practice-time interference. Leave both practice
switches off in matches. Detailed logging by itself never enables these early saves.

`BACKGROUND_STREAMING` retains the alternative of writing throughout a run. It intentionally
bypasses memory-only windows; select it before Play only when that behavior is wanted.
Storage mode is captured at Play. Buffer capacities apply when the run's buffer is created.
Other timing/practice flags may be adjusted live.

Both streams default on to collect the next round of tuning. For a comparison of overhead,
run the same practice sequence with detail on and off. Watch:

- `loop_period_ms`: elapsed time between logging samples, including the surrounding robot loop.
- `shooter_loop_ms`: actual period used by the shooter controller.
- `previous_capture_ms`: previous loop's logging capture/enqueue time (not disk-write time).
- DS `Log capture last/max ms`, buffered row count and dropped-row counters.

The logger reuses shooter diagnostics and the intake loop's cached beam samples. Detailed
capture additionally reads each intake/transfer motor's velocity once and each SDK power
readback once per recorded loop. Disabling detail avoids those extra reads. In memory mode,
capture retains scalar snapshots without formatting CSV. When saving is authorized, CSV
formatting, directory creation and disk I/O happen on a low priority background thread.
Checkpoints flush when their released batch is written; long batches also flush periodically.
The bounded buffer drops rows rather than exceeding capacity; part is reserved for basic rows.
Memory pressure never automatically authorizes writing inside a protected match window.
Drops and errors are visible in telemetry. The cap counts snapshots, not bytes; memory usage
depends on enabled fields, loop rate and run length. Use drop counters to verify the capacity
is sufficient for your measured loop rate, especially with detail enabled.
Robot performance still needs measurement on the Control Hub; desktop tests cannot establish it.

## Files and time alignment

Each Play reserves a unique folder name, created when its first save is authorized:

```text
/sdcard/FIRST/logs2026/YYYYMMDD_HHMMSS_mmm_OpMode_uniqueId/
    match.csv
    shot_detail.csv
```

A file is created only if its stream has data to save. New runs never overwrite older runs.
Use Android Studio Device Explorer or ADB to copy logs:

```powershell
adb pull /sdcard/FIRST/logs2026 Data/logs2026
```

There is no automatic deletion. Copy/archive and clean up old logs periodically.
Rows include schema version, unique session ID, OpMode, phase, profile, wall-clock epoch ms,
and monotonic `match_t_ms` since Play. Both streams use the same clock/sequence IDs.
Separate autonomous and teleop runs can be combined offline using wall-clock alignment.
Changing the hub clock between runs will affect that alignment.

The background saver drains after Stop without blocking motor shutdown. In default memory
mode, a process/hub restart before saving loses the buffered run, matching the previous
in-memory approach. Practice checkpoints/streaming preserve complete CSV rows already written;
the converter skips an incomplete final row. A newly started OpMode can overlap a prior run's
still-finishing save if started immediately after Stop.

## Basic data and event meanings

`match.csv` includes:

- Robot X/Y in **Pedro inches**, heading in **degrees**, field velocity and angular velocity.
- Autonomous stage/selected plan, partner delay, follower busy state, path index/progress and
  path heading goal. `path_t` is Pedro's reported value, not a measured arrival guarantee.
- Shooting target reference, selected RPM source, operator aim/fire request and TeleOp firing
  permission. Auto uses a reference goal for distance context, **not** a claim of active turret
  tracking; its fire-permission field is blank because it has no equivalent per-loop interlock.
- Tracked ball count, raw beam mask and latched occupancy mask, feeding state, sequence ID,
  sequence start time/count/beams, current elapsed time and completion/cancellation reason.
- Target/measured flywheel RPM, battery voltage, requested/applied boost, applied power,
  intake/flower state, turret mode/calibration state, measured/target/robot-relative angle.
- Logging timing, queue depth and drop counts.

Events include `SHOT_START`, `SHOT_END`, `BEAM_EDGE`, `BALL_COUNT`, `STAGE`, `FIRE_REQUEST`,
`OUTPUT_STATE`, `TURRET_STATE`, `CONFIG`, stream resume/toggle and session start/end.
Configuration snapshots include public primitive/string/enum settings from the current OpMode,
teleop model, shooter, feed controller, intake, turret, flower and logger, plus shot-table
settings and sample rows. They are checked about once per second and when a stream is enabled,
and saved independently in each enabled stream. RUN_NOTES and BUILD_LABEL therefore survive
with only detailed logging enabled. Detailed CONFIG rows contain common context and a note,
with blank extra measurements; use event=DETAIL for measured sample analysis. Detailed
PID/feed settings are captured every sample. Keep settings fixed within a trial; a brief
edit between configuration checks can be missed.

**SHOT_START means a volley/feed sequence was commanded, not that a ball launched or scored.**
The sequence start count is saved before feeding clears the intake's occupancy model. Current
ball count is that software model, not an independently verified count. Each beam mask uses:

| Bit | Value | Sensor/position |
| --- | --- | --- |
| 0 | 1 | At shooter |
| 1 | 2 | Middle high |
| 2 | 4 | Middle low |
| 3 | 8 | Intake entry |

For raw beams, 1 means **blocked** (the electrical digital channel is active low). Clear/block
edges can also come from holes or vibration; do not interpret every edge as a separate ball.
Sampling is once per robot loop, so transitions entirely between samples cannot be recovered.

## Detailed shot data

`shot_detail.csv` repeats the basic columns so it is usable without `match.csv`, plus:

- Shooter measured/encoder-delta RPM, current, error, integral, derivative, acceleration,
  individual P/I/D and feedforward terms, gains, gearing, slew and voltage compensation settings.
- `controller_target_rpm` records the target associated with the last controller update;
  the common `target_rpm` may already reflect a newer OpMode request. Compare sample age
  when aligning cached outputs to requests.
- Raw/filtered battery readings, compensation gain, clipping/slew status and sample age.
- Active feed/boost timing snapshot, boost state and power comparisons described below.
- Intake and transfer commanded velocity or power, control mode, measured motor/shaft RPM,
  SDK power readbacks, capacity-stop states, diagnostic/reverse mode and gate servo command.
- Four beam booleans and cumulative block/clear edge counts, plus tracked occupancy in the common columns.
- Cached turret encoder counts, absolute phase, commanded/measured/target angles, servo positions,
  learned offset, readiness/reachability and status. Unavailable measurements are NaN/blank;
  CHASSIS_ONLY never initializes turret hardware. FIXED mode has a commanded angle, not a measured one.
- Regression coefficients, table flag and operator RPM trim.

Intake/transfer `*_power_command` is NaN while velocity control is being commanded; velocity
command is NaN while power control is being commanded. The Hub's internal velocity-controller
PWM is **not** exposed by this logger. `*_controller_power_readback` is the SDK `getPower()`
readback, which must not be mistaken for measured effective PWM under `setVelocity()`.
Motor RPM and shaft RPM are separate; shaft RPM uses the subsystem's 3:1 reduction.

Cached applied controller data can trail a new OpMode request by one update. Shooter/beam
sample ages and requested-versus-applied boost fields make that distinction visible. Gate and
servo positions are commands; no physical gate-opening or servo-angle sensor is inferred.

### Interpreting boost

| Column | Meaning |
| --- | --- |
| `controller_power_before_boost` | Clipped PID + voltage-compensated feedforward, before boost/slew |
| `minimum_power_override` | Current requested minimum-power floor |
| `boost_requested_power_increase` | Increase caused by the floor before slew |
| `no_boost_output_same_loop` | Controller-only output through normal slew, starting at the same previous actual power |
| `applied_power` | Actual software power command sent to the shooter motor |
| `boost_applied_power_difference` | Actual command minus that same-loop no-boost comparison |

The same-loop comparison is **not** a simulation of a complete run without boost. Earlier boost
changes actual RPM and the previous power, so its later effects remain in the measured system.
Use a real A/B test to measure what disabling boost changes.

## AdvantageScope replay

Use the new converter for these logs, rather than last year's artifact/hood visualization:

```powershell
python scripts/convert_logs_2026.py --input "Data/logs2026/<run-folder>" --output Data/run.wpilog
```

It combines both CSV streams, exports all scalar fields under `/Match` and `/ShotDetail`, and
creates modern typed geometry under `/Replay`. No Python packages are required to convert.
In AdvantageScope open the `.wpilog`, add **2D Field** or **3D Field**, then select:

- `/Replay/RobotPose` for the robot (typed Pose2d, meters/radians).
- `/Replay/ShooterDirection` for its current robot-relative shooting bearing.
- `/Replay/CommandedVolleyLocations` for an accumulated list of commanded volley locations.
- `/Replay/TargetReference` for the target reference point.

Use line graphs for ball count, beam states, RPM, power, current and voltage. There are no
synthetic flying balls or inferred scores. Replay is visualization of measured/logged states;
it does not rerun the controller or automatically establish why a shot missed.

Default `--frame ftc` uses the same transform as this repository's previous converter:
`x_m=(72-y_in)*0.0254`, `y_m=(x_in-72)*0.0254`, heading = Pedro heading +90 degrees.
Use AdvantageScope's **Center/Rotated** coordinate system and verify the field asset orientation.
`--frame pedro` preserves bottom-left-origin Pedro axes, converting only inches to meters and
degrees to radians, for a matching custom field. Raw CSV coordinates are never transformed.

To join auto and teleop while retaining the transition gap:

```powershell
python scripts/convert_logs_2026.py --input "Data/auto-folder" "Data/teleop-folder" --timeline wall --output Data/match.wpilog
```

Different session IDs are rejected with the default run timeline to avoid accidentally
superimposing unrelated practice runs. `--timeline wall` explicitly joins them by epoch time.
Do not change the Control Hub clock between auto and teleop if using this alignment.

Sources: [AdvantageScope log formats](https://docs.advantagescope.org/getting-started/manage-files/),
[coordinate systems](https://docs.advantagescope.org/more-features/coordinate-systems/),
[modern geometry support](https://docs.advantagescope.org/overview/legacy-formats/),
[WPILOG specification](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/wpiutil/doc/datalog.adoc).

## First robot check

1. Enter a profile label identifying flywheel mass, preload technique and boost setup.
2. Run a short practice session, shoot, stop, and pull its folder.
3. Confirm CSV rows, nonzero RPM/voltage, correct four beam states, and correct replay orientation.
4. Compare detail on/off loop periods and capture time before choosing match settings.
5. Capture fresh-battery and used-battery opening volleys with otherwise identical setup.

No shooter gains, boost settings, voltage compensation behavior, feed timing, paths, turret
motion or firing interlocks were intentionally changed by the logging implementation.

For the shooting model, a phased test plan, repeatable camera setup and a detailed audit
of measured versus externally observed information, see [Shooting_Analysis_2026.md](Shooting_Analysis_2026.md).
The offline `shooting_model_2026.py summarize` command summarizes actual detailed logs by
volley without interpreting beam transitions as individual launches or scores.
