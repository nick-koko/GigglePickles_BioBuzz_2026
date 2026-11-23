# Automated stationary shooting experiments

## Next session

Run **Shot Opening Repeat 2026** for eight cold starts at 3100 RPM.
Preload the first set before Play; it fires automatically. Hand load each
remaining set and press Y when ready.

Use **Shot Running Batches 2026** for the running comparisons. During Init,
D-pad up/down selects the batch, left selects default order, and right selects
rotated order. Changing batch resets order to default. The selected batch,
order and automatic volley count appear on telemetry. A intakes and Y shoots.

The normal sequence requires no Panels changes. Each setting runs four
volleys automatically. Use Panels for target RPM or other tuning adjustments.

The opening program freezes one configuration. Feed is intake 0.50 and transfer
0.50 for 1.2 s. Flywheel boost is the original flat 0.75 floor, delayed 0.18 s,
lasting 0.50 s, bypassing slew. Boost voltage scaling and RPM taper are off.
Feedforward voltage gain is allowed from 0.90 to 1.10.

Then select **Shot Running Batches 2026**. The running RPM defaults to 3100.
The first fresh Y starts wheel preparation. Every volley gets the existing
1.5 s preparation plus a two-second fixed observation interval before feed.
After each volley the wheel stays running while waiting for four balls and
another fresh Y. A intakes and Y shoots. X stops the motors.
The default batch `BOOST_COMPARISON` has four volleys with the original
boost followed by four with the early ramped boost. Both share the 0.90..1.10
feedforward compensation and the same fixed target and feed commands.

Each group repeats the same setting four times, then the next Y-confirmed
volley uses the next setting. Telemetry shows the upcoming profile, target and
repeat number. The plan manifest records every override and trial ordering.
Trial rows record the profile, repetition, voltage, measured RPM, applied motor
commands and feed configuration. Abort/retry attempts retain the same trial ID.
Four repeats means four completed sequences; video/manual notes supply the
actual score and grouping.

| `ShotSession2026.RUNNING_BATCH` | Settings, in default group order | Total volleys |
| --- | --- | ---: |
| `BOOST_COMPARISON` | Original flat 0.75 at 0.18 s; model ramp 0.80 to 0.90 at 0.04 s with voltage scaling and 100 RPM taper | 8 |
| `BOOST_TIMING` | Model ramp at 0.04 s; same ramp at 0.08 s | 8 |
| `BOOST_LEVEL` | Model ramp 0.80 to 0.90; same controls with 0.75 to 0.85 | 8 |
| `FEED_SPEED` | Model boost with intake/transfer 0.50/0.50; 0.50/0.425; 0.425/0.50; 0.425/0.425 | 16 |
| `FEED_FAST` | Model boost with intake/transfer 0.50/0.50; 0.50/0.575; 0.575/0.50; 0.575/0.575; 0.65/0.65 | 20 |
| `RPM_BRACKET` | Model controls at running target minus 100, target, target plus 100 RPM | 12 |

Run `BOOST_COMPARISON` first. Use later batches to investigate one factor at
a time after reviewing the first result. All are already-spinning trials.
`VOLLEYS_PER_SETTING=4` controls running group size; `VOLLEYS=8` controls
only the opening program. `RPM_BRACKET_OFFSET` sets the bracket step.
`RUNNING_ORDER_OFFSET=1` rotates whole groups for a second run to reduce the
confounding effect of discharging the battery. Compare logged voltage between groups. A group of four is an initial
sample for variation, so retain outliers and repeat promising comparisons.

Use the first opening batch to measure repeatability before changing a parameter.
If the group is tight but consistently long/short, adjust RPM for a new batch.
If the group is wide, compare entry placement, release spacing, launch angle
and RPM near each contact. Record ball identities and launch order, and rotate
their order in a separate batch to distinguish a particular ball from a
particular place in the volley. Keep that change out of the first repeatability
batch. Capture a fixed side view of the wheel exit and a wide view of the hive,
with a known length in the flight plane. Count all four balls even when one
leaves the expected image region.

Assess the opener by complete four-ball success and worst-ball clearance,
alongside startup time. Assess already-spinning trials by four-ball spread at
the hive and successful shots across repeated volleys. The wheel model estimates
RPM response. Flight variation also depends on ball compression, spin, slip,
exit angle and collisions. A short fourth-ball gap can be successful and is
useful data when all four trajectories group well.

The four original **Autonomous → Test** programs also remain available:

- **Shot Recommended 2026** provides candidate-centered batches with preset likely-useful
  settings and nearby variations. Start here for the next tuning session.
- **Flywheel Model 2026** collects unloaded startup/coast and target-transition data at
  several RPMs. It never starts automatic feeding; run with the robot empty.
- **Shot Compare 2026** cycles named parameter candidates under cold and already-spinning conditions.
- **Shot Repeatability 2026** repeats one selected configuration from stopped, already at target,
  below target and above target. Default: three repeats of each condition (12 volleys).

They initialize only the shooter, intake/transfer, break beams, gate and LEDs. Required
hardware is resolved during Init, as in the normal subsystems. There are no chassis,
odometry, turret or flower commands. Manually place and aim the robot at a consistent
shooting location. These are practice programs, not field autonomous paths.

## First 30 minutes for a model comparison

For a loaded autonomous opener test, mark the actual opening pose and set
`TARGET_RPM` to the team's best scoring estimate for that pose. The November 22
screen at 3196 RPM generally shot long. Repeat that command when the purpose is
to compare models against that dataset. For opening speed and scoring, use the
intended match RPM and measure Play to feed, Play to fourth exit, and grouping.

1. **0–5 min.** Mark a fixed pose and camera position. Fit a fresh battery, manually
   preload four balls as for autonomous, and leave collection off. Keep detailed logging on.
2. **5–15 min.** Run **Shot Recommended → QUICK_SCREEN** at **3196 RPM for every profile**.
   Record the feed LED and ball exits in the same video. Call out each trial ID, any
   unusual preload, and balls remaining. Press Y after each refill to four total.
3. **15–25 min.** Empty the robot. Run **Flywheel Model → STARTUP_AND_COAST**, then Stop,
   select **STEP_RESPONSE** in Panels and restart. Press Y once per batch. Logs suffice
   for these unloaded tests.
4. **25–30 min.** Save the three run folders and the loaded-shot video. On a later
   fresh battery, set `PROFILE_ORDER_OFFSET=1` for Quick Screen so the modeled
   candidate receives the first manual preload. Record actual voltage for each volley.

**Compare controller tracking at a common command.** Use time from spinup to sustained
±50 RPM, peak overshoot, actual RPM immediately before feed, and RPM through the last
visible exit. The 3196 RPM command is a tracking reference. Each Quick Screen condition
has one volley, so repeat promising settings before choosing between them.

**Compare the old opener by its measured speed.** The team used a 2950 RPM command
because the original controller overshot and decayed slowly enough to feed while the
wheel was still faster than 2950. Align the old and new wheel-speed traces at the feed
command and keep their respective target RPMs visible. Use measured RPM at feed and
through the volley. The historical logs lack video of individual exits. Once sag and
startup are repeatable, use `LOADED_RPM_BRACKET` and scoring video to choose the target.

The six league openers commanded 2950 RPM. Their last encoder samples before feed
were **3129–3171 RPM**, taken 17–28 ms before the command. Their pre-feed peaks
were **3343–3386 RPM**. Keep those values as observations of the original mechanism
and controller. Each bound is rounded to the nearest RPM.

To compare a practice folder with those openers, run the following from the repository root
(Python with NumPy and SciPy). Supply either folder alone to analyze just that session or cohort.

```powershell
python scripts/compare_flywheel_trials_2026.py "C:/Users/nicko/Downloads/logs2026/LeagueMeet_11-19-2026" "C:/path/to/practice" --output build/flywheel-comparison
```

`trial_summary.csv` identifies attempts, outcomes, pre-feed measurements and sample timing.
For loaded cold trials, settling time starts at the first sampled positive command and
requires a final uninterrupted measured run within ±50 RPM lasting at least 250 ms before feed.
A blank settling time means the log did not establish that condition.
`rpm_traces.csv` includes feed-relative timestamps for loaded trials, recorded power,
voltage and the frozen plant model's predicted encoder response to those inputs.
Review completed practice attempts separately from interrupted ones. Score the
existing fit before changing it. Compare unloaded startup/coast first and reserve
step response as a check on any refit. For loaded shots, mark the first and last
visible exits against the white cue. The replay uses the original empirical load
timing and assumes the league wheel with four weights. The exported time axis includes
the recovery period so the end of the analysis window can be chosen from video.

## Recommended preset tests

Select **Shot Recommended 2026** and use `RECOMMENDED_BATCH` in Panels. Default
`QUICK_SCREEN` is eight volleys: current settings, the explicit modeled reference,
lower boost, and later boost, each from stopped and already spinning. No manual
coefficient entry is needed. Set the location's `TARGET_RPM`, run label and battery label.

The reference is a hypothesis from the league data/model, not a proven optimum:

| Parameter | Explicit test reference |
| --- | --- |
| PIDF | P 0.0009; I/D/A 0; kS 0.09; kV 0.000134 |
| Startup slew up / down | 6 / 6 power per second |
| Running slew up / down | 1.5 / 1.5 power per second |
| FF voltage compensation | 12.5 V reference, gain 0.90–1.10; 0.5 s filter |
| Timed boost | Voltage-scaled 0.80→0.90 over 0.50 s, starting 0.04 s after feed |
| Boost overspeed taper | 100 RPM; boost bypasses slew |
| Feed | 1.2 s; intake/transfer power 0.50 / 0.50 |

Startup uses the existing 200 RPM entry threshold, 50 RPM tolerance, 200 RPM/s
exit-slope limit, 0.15 s stable interval and 2 s timeout. Each candidate overrides
the reference only as listed below. Current settings in the quick screen inherit
the production snapshot instead. This keeps future production edits from silently
changing the meaning of the modeled reference.

| `RECOMMENDED_BATCH` | Variations | Default volleys |
| --- | --- | ---: |
| `QUICK_SCREEN` | Current; reference; boost 0.75→0.85; delay 0.08 s | 8 |
| `STARTUP_NEIGHBORS` | Reference 6/6; 3/6; 6/3; 10/6 startup slew | 8 |
| `BOOST_LEVEL` | 0.75→0.85; 0.80→0.90; 0.85→0.95 | 6 |
| `BOOST_TIMING` | Start at 0 / 0.04 / 0.08 s after feed | 6 |
| `BOOST_TAPER` | 0 / 100 / 200 RPM overspeed taper | 6 |
| `RUNNING_NEIGHBORS` | Running slew 1.5/1.5; 1.5/3; 3/3, each warm / +200 / −200 RPM | 9 |
| `LARGE_TRANSITIONS` | Running 1.5/1.5 vs 1.5/3; +450 and −450 RPM transitions | 4 |
| `FEED_NEIGHBORS` | Reference; transfer 0.425; intake 0.425; both 0.425; both 0.350 | 10 |
| `LOADED_RPM_BRACKET` | Target minus 100 / target / target plus 100 RPM | 6 |
| `P_GAIN` | P 0.0007 / 0.0009 / 0.0011 | 6 |
| `FEEDFORWARD_GAIN` | kV 0.0001273 / 0.000134 / 0.0001407 (±5%) | 6 |

Except the two transition-specific batches, each candidate runs cold and already
spinning. `RECOMMENDED_COLD` and `RECOMMENDED_ALREADY_SPINNING` select these conditions.
The transition batches use their stated conditions regardless of those two flags.
`RECOMMENDED_REPETITIONS` defaults one. `PROFILE_ORDER_OFFSET` and repetitions rotate
candidate order, including RPM order in the bracket batch. The ±200 transition is
`STEP_DELTA_RPM`; the ±450 transition is `MODEL_LARGE_STEP_RPM`. Both finish at the
same `TARGET_RPM` before feeding. The bracket deliberately changes shooting RPM at
a fixed location to measure scoring/RPM sensitivity; `LOADED_RPM_BRACKET_OFFSET` sets it.

Suggested sequence:

1. Run `QUICK_SCREEN` with a fresh battery and again at a representative lower loaded
   voltage. Rotate the first candidate on subsequent battery swaps; one battery's
   entire batch cannot reproduce the very first fresh-battery condition for everyone.
2. Use `STARTUP_NEIGHBORS` if startup still overshoots; use boost level/timing/taper
   batches to investigate RPM through ball contact. Compare the video-defined contact
   window and made/short/long outcomes, not just post-volley overspeed.
3. Run running-transition and feed batches next. Keep feed duration long enough to
   empty the robot; use `TEST_FEED_SECONDS` if needed. A changed feeding speed needs
   a new observed ball-contact load profile for the model.
4. Bracket RPM once regulation is reasonably consistent. The former 2950 RPM opening
   workaround and the 3196 tracking reference are not assumed ideal scoring RPMs.
5. Use **Shot Repeatability 2026** with `MODELED_CANDIDATE` or `CUSTOM` for repeat
   verification. Copy chosen non-reference settings into `CUSTOM_*` for that verification;
   selecting a recommended batch does not automatically update the custom profile.

Do not combine every batch into one marathon run: battery drift and ball handling
would dominate comparisons. Change one batch at a time and repeat promising candidates.
P/kV batches test model/controller sensitivity; they are lower priority than startup,
boost and feeding tests. All presets remain confined to the test opmodes.

## Unloaded model data

Select **Flywheel Model 2026**. Remove all balls, then use one fresh **Y** to confirm
empty/start the batch. The remaining unloaded trials advance automatically after a
fixed `MODEL_BETWEEN_TRIAL_SECONDS` interval (default 1 s), in addition to each trial's
preparation, hold and coast-down checks. The interval is captured at Play and recorded
in `TEST_PLAN`; `BETWEEN_UNLOADED_TRIALS` phase timestamps record actual loop timing.
A collection and controller 2 A override are disabled. X or B interrupts automatic
continuation and stops the motors; a fresh Y is required to retry/resume. A timeout
also pauses until a fresh Y. Manual skipping while paused leaves the program waiting
for confirmation. B manual reverse remains available for clearing while waiting/paused.
Confirm physically empty before starting or retrying the batch. Loaded-shot programs
still require fresh Y for every volley.

`MODEL_BATCH` selects:

- `STARTUP_AND_COAST` (default): three trials, at 2800 / 3200 / 3600 RPM. Each begins
  from measured near-zero, records 2 s of startup plus 2 s holding the target, then
  commands zero power and records coast until measured RPM is at most 50 for 0.10 s.
  Coast timeout pauses with `COAST_CAPTURE_TIMEOUT`; it never pretends the wheel stopped.
- `STEP_RESPONSE`: eight trials ending at 3200 RPM, starting at ±200 and ±450 RPM
  from that target, each as an immediate step and as a 1 s target ramp. Prepare at
  the initial target for 3 s, observe the transition for 2 s, then hold for 2 s.
  The motor stays enabled between these trials. Actual initial RPM is logged separately;
  fixed preparation does not promise the requested initial RPM was achieved.

These use the explicit reference controller with timed boost disabled. `MODEL_RPM_*`,
`MODEL_SMALL_STEP_RPM`, `MODEL_LARGE_STEP_RPM`, `MODEL_STEP_RAMP_SECONDS`,
`MODEL_PREPARE_SECONDS`, `MODEL_OBSERVE_SECONDS`, `MODEL_HOLD_SECONDS`, `MODEL_BETWEEN_TRIAL_SECONDS` and
`MODEL_REPETITIONS` configure only these plans. Coast uses the normal test timeout settings.
RPM/order rotation works as in the loaded batches. The RPMs bracket likely useful
operation; no-load testing at a particular RPM does not recommend shooting there.

Repeat at fresh, middle and lower practical battery conditions. This data can refine
power/voltage-to-RPM response, inertia/drag estimates, sensor lag and controller
transition behavior without conflating them with unknown ball-contact forces. Loaded
volleys are still needed to identify the contact loads and judge scoring. No-load
bench voltage can differ from voltage while driving; use the measured voltage trace.

Set `FLYWHEEL_WEIGHT_COUNT` and `FLYWHEEL_DIAMETER_MM` to the physical configuration.
They are logged metadata, not an automatic model or controller adjustment.

## First loaded-shot run

1. In Panels open **ShotExperimentSettings2026**. Choose `TARGET_RPM` for the actual
   shooting location. The default 3196 is a model tracking reference, not a proven
   ideal scoring RPM. Keep this final RPM and position constant across candidates.
2. Set `RUN_LABEL` to identify hardware/mass/video and `BATTERY_CONDITION` to `FRESH`,
   `MID`, `LOW` or another useful label. This label never substitutes for measured voltage.
3. Choose a recommended batch, comparison set or verification profile. Settings and the complete
   schedule are captured at Play; change settings before Play and restart to rebuild a plan.
4. Load **four total balls**, press Play, then press **Y** to confirm the load and begin
   the trial. A Y already held when Play is pressed must be released and pressed again.
5. After the volley and recovery recording interval, refill to four total and press Y
   again. The routine never starts another shot solely because beams indicate full.
   Check for remaining balls before adding four more, especially in slower-feed tests.
6. At COMPLETE the motors stop, logs save, and all borrowed normal configuration values
   are restored. Press Stop. Stop and unexpected exceptions also run cleanup and restoration.
   X pauses within the test; settings remain scoped until the next trial or Stop.

During a trial, the robot feeds after a fixed delay. It deliberately does not wait
for flywheel RPM to reach target, so a poor candidate cannot hide behind extra settling time.
Cold preparation has a separate bounded wait for RPM to fall near zero; failure pauses the test.

## Controls

| Control | Action |
| --- | --- |
| Fresh Y on either controller | Confirm four loaded and start/retry the displayed trial |
| A on either controller while waiting/paused | Start collection |
| Hold controller 2 A while waiting/paused | Intake-only capacity override; transfer capacity logic remains active |
| Hold B while waiting/paused | Reverse for manual clearing |
| B during a timed trial | Abort that attempt, then allow manual reverse |
| X | Cancel feed, stop motors and pause; a fresh Y starts a new attempt of that trial |
| D-pad right while waiting/paused | Skip the upcoming/current trial |
| Stop | End the run, save logs and restore configuration |

Intake controls are ignored during automatic spin-up/feed/recovery, except X/B abort.
Holding Y cannot queue another volley. Retries have unique attempt IDs; sequence IDs
also remain monotonic through a run. Paused/skipped attempts are identified in the logs.

## Comparison groups

`COMPARISON_SET` selects a small controlled matrix. Unlisted parameters inherit the
normal configuration captured at Play. A trial always starts from that snapshot,
so the previous candidate cannot leave an unrelated setting behind.

| Set | Candidates |
| --- | --- |
| `STARTUP` (default) | Current; startup 1.5 up / 6 down; startup 6 / 6; startup 10 / 6 |
| `VOLTAGE` | Current; FF gain range 0.90–1.10; voltage-scaled boost; both |
| `BOOST` | Current; boost disabled; earlier flat boost at 0.04 s; earlier 0.80→0.90 ramp with 100 RPM overspeed taper |
| `FEED_POWER` | Current; transfer-only mild reduction; intake-only mild reduction; both mild; both slow |
| `CUSTOM_VS_CURRENT` | Current versus the `CUSTOM_*` settings in the test class |

`COMPARE_COLD` and `COMPARE_ALREADY_SPINNING` default true. Most groups therefore
have eight volleys per repetition; feed power has ten, custom versus current has four.
`COMPARISON_REPETITIONS` defaults one. Increase it for repeat data, or run shorter groups
at several battery conditions. This is not a blind cross-product of every parameter.

`PROFILE_ORDER_OFFSET` rotates which candidate runs first; change it on successive
battery sessions. Order also rotates across repeated blocks. This reduces the bias
from always giving the baseline or one candidate the freshest battery. Actual voltage
will still change during a batch; analyze it as measured rather than treating a whole
batch as one voltage. Only the first trial after a battery change has that initial condition.

## Repeatability profiles and starting conditions

`VERIFICATION_PROFILE` selects:

- `CURRENT_SETTINGS` (default): the normal configuration captured at Play.
- `MODELED_CANDIDATE`: startup 6/6, bidirectional FF gain range 0.90–1.10,
  voltage-scaled early 0.80→0.90 boost with a 100 RPM overspeed taper.
  This is an unvalidated model candidate, not a scoring guarantee.
- `CUSTOM`: uses test-only `CUSTOM_*` PIDF, slew, voltage, boost and feeding settings.

Enable/disable conditions using `VERIFY_COLD`, `VERIFY_ALREADY_SPINNING`,
`VERIFY_STEP_UP` and `VERIFY_STEP_DOWN`. All default true.

| Condition | Preparation and trigger |
| --- | --- |
| `COLD` | Stop/coast to at most 50 RPM for 0.10 s; enable final target; feed after 2.0 s |
| `ALREADY_SPINNING` | Command final RPM for 1.5 s preparation, then feed after a further 2.0 s |
| `STEP_UP` | Prepare at final minus 200 RPM for 1.5 s; ramp to final over 1.0 s; feed 2.0 s after transition begins |
| `STEP_DOWN` | Same, starting 200 RPM above the final target |

Those values are adjustable in the test settings. `TARGET_RAMP_SECONDS=0` makes an
instantaneous target step. Actual RPM at the transition is recorded separately from
the requested starting target; a warm trial can reveal inadequate preparation instead
of silently waiting longer. All conditions shoot at the same final RPM and location.
Verification condition order rotates across repetitions.

`COAST_TIMEOUT_SECONDS` defaults 20. If the wheel never falls below the stopped threshold,
the routine pauses without feeding. It does not reset an encoder to fake zero. Between
trials it keeps the shooter running for a warm next condition, or stops it early so it
can coast during reload before a cold next condition.

`POST_FEED_CAPTURE_SECONDS` defaults 1.0. No automatic loading occurs during this tail.
Late unloaded recovery is useful model data but is not evidence that a shot went long.

## Feeding power tests

Normal collection uses motor RPM commands (currently intake 4000 / transfer 700).
Shot feeding calls `setPower`, currently 0.50 / 0.50, while the motors remain
in `RUN_USING_ENCODER`. The Hub interprets these as fractions of the configured
maximum motor velocity and regulates speed. The logged POWER label identifies
the API command. Actual acceleration, ball slip and motor loading still affect
ball release spacing.

To test faster feeding, select `ShotSession2026.RUNNING_BATCH=FEED_FAST`.
This runs five groups of four volleys, all with the same target RPM, model
candidate boost and 1.2 s feed:

| Group | Intake command | Transfer command | Volleys |
| --- | ---: | ---: | ---: |
| Baseline | 0.50 | 0.50 | 4 |
| Faster transfer | 0.50 | 0.575 | 4 |
| Faster intake | 0.575 | 0.50 | 4 |
| Both moderately faster | 0.575 | 0.575 | 4 |
| Both faster | 0.65 | 0.65 | 4 |

The 0.575 and 0.65 settings request 15% and 30% higher motor velocity than
baseline. Verify actual speeds during ball movement from the logs. Score
four-ball grouping, time to fourth exit, release spacing, and RPM dips. A
shorter gap is acceptable when all four trajectories group consistently.
Faster feeding could also move ball contact earlier relative to the fixed
boost window. Record that timing before interpreting a result as a motor
power or inertia limit. The model's load timing and energy draw should be
refitted using this evidence instead of assuming unchanged pulses.

For the first feeding investigation after the boost comparisons, prioritize
`FEED_FAST`. The separate `FEED_SPEED` batch remains available for the
slower-feed comparison. Each includes a baseline at the beginning so it can
be assessed with the battery conditions of that run.

For the slower-feed comparison, select **Shot Running Batches
2026** and `ShotSession2026.RUNNING_BATCH=FEED_SPEED`. Its four groups are
baseline, transfer-only reduction, intake-only reduction and both reduced,
for 16 volleys total. Use normal A collection and a fresh Y after confirming
four balls each time. Target RPM, candidate boost and feed duration are the
same across groups. At 0.425 the requested motor velocity is 15% below the
0.50 command. Record all four exits and any balls left after the 1.2 s feed.
Compare grouping, time between releases and measured feeder RPM during the
volley as well as flywheel RPM. Pause or stop after any completed group if
time is limited; partial runs can be analyzed from their completed trial IDs.

`Shot Compare 2026` also provides its original `FEED_POWER` comparison:
`FEED_MILD_SCALE=0.85` and `FEED_SLOW_SCALE=0.70` scale the captured shot powers.
With today's baseline, the feed matrix is:

| Candidate | Intake feed power | Transfer feed power |
| --- | ---: | ---: |
| Current | 0.500 | 0.500 |
| Transfer mild | 0.500 | 0.425 |
| Intake mild | 0.425 | 0.500 |
| Both mild | 0.425 | 0.425 |
| Both slow | 0.350 | 0.350 |

Actual motor RPM and SDK power readbacks are logged every detailed sample, alongside
the API commands, battery voltage, beam states and flywheel response. The motor
controller's power readback in velocity mode is not measured electrical duty.
Velocity feedback cannot guarantee the same acceleration or ball motion at
different battery conditions.

`TEST_FEED_SECONDS=0` inherits each profile's feed duration (normally 1.2 s). A positive
value overrides duration for every trial in the run without editing normal opmode
settings. `CUSTOM_FEED_SECONDS` and the two `CUSTOM_FEED_*_POWER` values can instead
define a custom candidate. Do not assume every slower trial empties all four balls
within the existing timer: inspect/video the result before the next refill.

Changing feed speed can change the spacing and duration of ball contact loads. The current
model's fixed empirical four-lobe profile must be checked/refitted for these trials;
evaluate the whole observed volley, not just the original 500 ms timing assumption.

## Logs, video and battery checks

The existing `match.csv` and `shot_detail.csv` contain trial ID, unique attempt ID,
profile, condition, repetition, phase, run/battery labels, operator-confirmed initial
load, requested initial/final target, actual initial RPM, ramp and feed delay.
Those metadata are also supported by the CSV-to-AdvantageScope converter.
Normal controller output, slew/boost stages, motor commands/velocities, break beams,
counts, voltage and current remain available. Test configuration is included in CONFIG rows.

Events include `TEST_TRIAL_START`, `TEST_PHASE`, `TEST_FEED_CUE` and explicit complete,
aborted/skipped markers. `test_confirmed_load=4` is the operator's confirmation, not a
sensor-verified physical count. Completion means the timed protocol ended, not four scores.
`TEST_PLAN` records the frozen complete schedule and explicit profile overrides.
`test_feed_enabled=false` and `test_confirmed_load=0` identify unloaded trials; their
`UNLOADED_HOLD` and `COAST_CAPTURE` phases generate no feed/shot event. In unloaded
trials, the legacy `test_feed_delay_s` column is the observation interval before the
unloaded hold, and does not describe an actual feed command.

LED markers: **double green at trial confirmation, double white at feed command**.
Record both LEDs and the ball path from a fixed camera position. The white marker is
not measured first-ball contact or exit; use the video frames to identify those.
Include trial IDs in your made/short/long observations so outcomes can be joined later.

`ENABLE_DETAILED_TEST_LOGGING` defaults true and records every observed loop.
The tests temporarily enable basic logging and select detailed logging from this flag.
Existing memory-until-Stop policy and 125 s protected tuning window are retained.
Long practice checkpoints may occur after that window if enabled in normal logging
settings. All prior logging flags, intervals, labels and LED cue settings are restored.

Run short groups after a battery change, partway through practice, and before changing
the battery. Useful previous-log checks were approximately 11.8 V typical late teleop
and 11.4 V lower-range operation; compare actual loaded voltage, not resting voltage alone.

Optional `TEST_X_INCHES`, `TEST_Y_INCHES`, `TEST_HEADING_DEGREES` and paired `AIM_X/Y_INCHES`
record a manually measured fixed location. Leave unknown values NaN; the test does not
invent a pose or use odometry. Record flywheel weight changes in `RUN_LABEL`/normal log notes.

## Isolation and desktop verification

There are no changes to default production PIDF, slew, boost, intake or transfer settings.
The test takes a configuration snapshot at Play, restores it before applying each
candidate, and restores it on completion/Stop/exceptions. Test settings themselves
are separate Panels values and are never read by normal opmodes.

Do not edit shared subsystem settings midway through an active trial. The schedule
and candidates are frozen at Play; direct external edits to globals would still
affect controller code and appear in detailed/configuration logs.

Run `.\scripts\run_shooter_configuration_checks.ps1`. It includes deterministic test
coverage for plan rotation, cold/warm transitions, fixed feed delays, ramp direction,
held buttons, retries/skips, coast timeout, parameter isolation/restoration, feeding
power profiles, log metadata and conversion, plus the prior controller/counting checks.
Desktop tests cannot validate ball travel, scoring or Control Hub loop timing.
