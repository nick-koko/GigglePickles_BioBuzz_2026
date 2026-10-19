# Shooting consistency comparison — October 15, 2026

[Repository list and rerun instructions](../team-code-comparison.md)

## Direct answer

**No explicit automatic increase in shooter power or RPM after the first ball was found in the external shooting implementations reviewed.** They use constant velocity targets, ordinary feedback control, and feed/gate timing. Feedback may increase motor output when a ball slows the wheel; that differs from detecting ball 1 and deliberately scheduling extra power for ball 2.

The clearest ideas worth adapting are:

- FTC-23511: wait for target speed before beginning the feed window, then latch the feed window so the first RPM dip does not interrupt it.
- Hazmat: use a tight threshold to enter READY and a looser threshold to leave it, reducing state chatter.
- Silver Circuits: keep the flywheel running and stop the intake motor while continuing the inner feeder during TeleOp shooting. This is a useful feed-load experiment, with effectiveness on our mechanism still unknown.

**Our local ShooterTuning2026 already implements a stronger explicit timed boost than any external code found:** after 0.20 seconds from feed start, it requests a minimum power of 1.0 with slew bypass until the default 3.0-second feed window ends. This is a timer experiment, not confirmation that ball 1 has passed. RPM-drop detection is telemetry only; it does not trigger or end the boost. The minimal 2026 TeleOp does not call this boost.

Sources: [FTC launch command][ftc-launch], [Hazmat readiness][haz-launcher], [Silver Circuits shooting state][vc-tele], and [our tuning implementation](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/ShooterTuning2026.java#L278).

## What was compared

Static source inspection of freshly fetched remote branch tips and the current local working tree. No robot runs, external builds, match footage, or shot logs were used to establish performance. Recommendations below are experiments, not verified improvements.

Our branch: **add-limelight-pollen**. HEAD: **0ec7274e4ea55ba6f10ad3fc4cfb50cd498f41bb**. The working tree contains existing changes and untracked 2026 shooter/TeleOp files; HEAD alone does not reproduce this local baseline.

| Repository | Branches deeply inspected | Pinned commit |
| --- | --- | --- |
| FTC-23511 | master baseline | d74ae8dbd1e50f2c116ff96c0a88cc774860962f |
| FTC-23511 | starter-bot-subsystems: launcher, Launch command, transfer, constants, TeleOp and tuner | e2e2e86d1076717105ae16c76fde906a44ca1d38 |
| FTC-23511 | pedro-brandon: newest tip, launcher not instantiated; pedro3: empty launcher shell | cecfd46190083b0058c851c528a3708bd4225dfc; 0c2345cf7345b4a001a63b2fd4d32acd891ca093 |
| VCSilverCircuits | master: scrim TeleOp, scrim autonomous, outreach TeleOp | 2ba1221a4c1edf5a948c30d5ee8d2d7c2c149429 |
| Hazmat | main baseline | ed074779446436618d81e56b37a4fed6ef50e5f1 |
| Hazmat | launcher: launcher, hood, calculation table, robot integration, tuning OpModes | 9166ce0088bd25a9e0c9174b9356e078f937b592 |
| Hazmat | transfer: transfer motor and sensing/queue | fd83188da19c28bce9480c59305f8364a59552f3 |

All advertised branch tips were inventoried and screened for shooting-related source. FTC-23511 branches: master, dev, pedro-brandon, pedro-brandon-2, pedro3, starter-bot-subsystems. Hazmat branches: main, PI1, autos, drivetrain, intake, launcher, next-testing, transfer, turret. Silver Circuits advertised master only. Deep review covered the distinct relevant implementations above; older history and every unrelated file were outside scope.

The newest FTC-23511 branch is **pedro-brandon (September 20)**, but the useful shooter code is **starter-bot-subsystems (September 13)**. Hazmat's newer transfer work does not establish that the launcher branch is merged. This is why neither “main only” nor “newest only” is sufficient.

## Comparison at a glance

| Behavior | Our 2026 minimal TeleOp | Our 2026 shooter tuner | FTC-23511 subsystem branch | Silver Circuits scrim | Hazmat launcher branch |
| --- | --- | --- | --- | --- | --- |
| Flywheel control | Software PID + feedforward, one motor | Same subsystem | Software PIDF, two motors grouped, one encoder feedback | TeleOp uses hub velocity PIDF | NextMotor velocity setpoint, follower motor |
| Initial feed waits for measured RPM | No | No; at-speed is displayed | Yes in Launch command | No in TeleOp; auto uses a 4-second wait | READY state exists; integrated shoot/feed gating not demonstrated |
| Feed behavior | Held trigger opens gate and runs intake/transfer | Timed 3-second feed | Latched 750 ms after first ready reading | TeleOp: intake off, inner feeder on; auto differs | Separate transfer branch manages storage |
| Explicit boost after ball 1 | No | 0.20-second delay from feed start, not ball detection | None found | None found | None found |
| RPM-drop-triggered recovery boost | No | Drop timestamp recorded only | None found | None found | None found |
| Readiness hysteresis | No ready latch | No ready latch | Single tolerance | None found | Enter at <=25 RPM error, exit above 75 RPM |
| Important qualification | Manual feed can start with shooter off or below speed | Full-power floor persists even after RPM recovers | Gains labeled untuned | Scrim auto does not show TeleOp's motor-mode/PIDF initialization | Gains, close/far speeds, and shot lookup values are zero placeholders |

Table evidence: [our minimal TeleOp](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Pickles2026Teleop.java#L159), [our tuner](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/ShooterTuning2026.java#L278), [FTC launcher][ftc-launcher], [FTC command][ftc-launch], [FTC constants][ftc-constants], [Silver TeleOp][vc-tele], [Silver auto][vc-auto], [Hazmat launcher][haz-launcher], [Hazmat calculation table][haz-calc], [Hazmat transfer][haz-transfer].

## Findings and what they mean for us

### 1. Wait for speed before starting a burst; latch the burst once it starts

FTC-23511's Launch command selects a near/far velocity, waits for readyToLaunch(), and then starts the transfer. The feed continues for 750 ms without rechecking readiness. A 2000 ms spin-up timeout ends a launch that never becomes ready; the feeder stops on command end. The flywheel stays running for repeat shots. Manual feeder overrides bypass this interlock. The tuner also drives the transfer directly despite its comment suggesting an at-speed condition. [Command][ftc-launch], [manual bindings][ftc-tele], [tuner bindings][ftc-tuner].

Our minimal TeleOp and tuner start feed from the trigger without an RPM interlock. **The best first adaptation is a pending-shot request that waits for valid measured speed, then starts the burst timer and boost timer together.** Require a nonzero target, a fresh controller sample, and a short stable interval rather than a single noisy reading. Add cancellation and a finite wait timeout. Our isAtSpeed() uses cached error, so calling it immediately after enabling must not accept stale “ready” data.

For continuous bursts, latching avoids stuttering feed after ball 1. Waiting for recovery between individual balls is a separate slower-feed experiment, not something FTC-23511 implements. Its velocity units are ticks/second; copy the sequencing idea rather than its numbers. [Launcher][ftc-launcher], [our speed check](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem2026.java#L417).

### 2. Separate “ready to enter” from “too far off to remain ready”

Hazmat updates absolute RPM error, enters READY at <=25 RPM, and leaves READY above 75 RPM. This hysteresis can make readiness indicators and pending-feed decisions more stable near a threshold. It does not itself add energy after a shot. Their isAtVelocity() helper uses the tighter threshold; it is not the same as the state-based isReady(). [Readiness implementation][haz-launcher].

**Adapt the state pattern, with tolerances measured on our encoder/loop.** The supplied launcher starts with zero gains and zero near/far speeds; the distance-to-speed table also contains only zeros. This is evidence of an architecture under development, not a proven tuned shooting solution. [Launcher defaults][haz-launcher], [lookup table][haz-calc].

### 3. Our boost experiment needs an explicit end condition

The tuner starts its feed timer on a trigger edge. At the default 0.20-second delay it sets minimum power to 1.0, bypassing the normal slew limit. It clears the floor only when the feed window finishes, is cancelled, or becomes inactive. Since controller output is clipped to 1.0, this holds full output even if RPM reaches or exceeds target. [Tuner](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/ShooterTuning2026.java#L291), [power floor application](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem2026.java#L202).

**For the next experiment, compare a bounded recovery pulse with the current long floor.** Trigger it from a validated near-shooter event or RPM sag after stable spin-up, with a timer fallback when sensors miss a ball. End the pulse on recovery/overspeed or a maximum duration and return to normal feedback. Track ball/event confidence before trying separate compensation for ball 2 and later balls. A feed-start delay alone cannot prove which ball contacted the wheel.

This proposal combines our existing control features with the external sequencing ideas; it is not a strategy found in the other teams' code.

### 4. Check slew limiting and power headroom before increasing the target

Our shooter rate-limits output at **1.5 power units/second** outside a bypassed override. A requested change from 0.60 to 1.00 therefore needs about **267 ms**, potentially spanning multiple closely spaced balls. Increasing gains or RPM can still leave actual applied power rising slowly. Compare requested output, applied output, and saturation during each shot. [Controller](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem2026.java#L192).

At the minimal TeleOp's 3855 RPM target, current defaults give feedforward of about **0.607** (0.09 + 0.000134 × 3855). With kP=0.02 per RPM, only about **20 RPM** of positive error drives the pre-slew command to full power. This calculation assumes default gains and zero I/D/acceleration terms; it is not a measured instability. Verify gain scaling and actual logs before layering on stronger boosts. Once applied output is already 1.0, another multiplier cannot supply additional motor power.

Voltage compensation is implemented but defaults **off**. Test across battery conditions; do not assume consistent loaded voltage or that software compensation can recover energy after output saturates. [Defaults and controller](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem2026.java#L28).

### 5. Test feed load as well as flywheel output

Silver Circuits' scrim TeleOp spins the wheel continuously at 2450 RPM, opens its ball stop while shooting, stops its intake, and keeps the inner feeder running. Their outreach variant uses 2000 RPM. Scrim auto uses a 4-second initial wait and 3.5-second shooting window at a constant target. None schedules a per-ball boost. [Scrim TeleOp][vc-tele], [outreach][vc-outreach], [auto][vc-auto].

Our shooting mode runs both intake and transfer at power 1.0. An A/B test with reduced/stopped intake while retaining transfer could change ball pressure, spacing, and electrical load; whether it helps depends on our geometry. Measure throughput and stalls as well as accuracy. Preserve continuous transfer as the initial baseline. [Our feed outputs](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeWithSensorsSubsystem2026.java#L177).

Do not assume every apparent external feature is active: the Silver TeleOp applies PIDF coefficients during init; editing its coefficient object in loop does not show another call applying them to the motor. Its autonomous file also lacks the explicit RUN_USING_ENCODER/PIDF setup visible in TeleOp. Hazmat's increasePower() is a manual tuner action, not a ball-triggered boost. Its transfer queue records arrivals, not proven flywheel-contact events. [TeleOp][vc-tele], [auto][vc-auto], [Hazmat tuner][haz-test], [transfer][haz-transfer].

### 6. Our older path already contains event-assisted burst recovery

The legacy ShooterSubsystem has hybrid ball-contact phases, breakbeam-window handling, timer fallback, and RPM/current-based ball-1 trigger options. It activates boost as those phases advance. Existing [burst notes](../../scripts/shooting_logging_and_burst_improvements.md) describe the experimental timing/logging workflow.

This is separate from ShooterSubsystem2026 and its single-motor tuner. Legacy code assumes different hardware and sensors; port only the needed ideas after measuring the new feed latency. The new intake currently defaults USE_TOP_BREAK_BEAM to false, so do not assume its beam is already controlling or counting shots. [Legacy phase logic](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java#L986), [legacy boost application](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java#L471), [new intake defaults](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeWithSensorsSubsystem2026.java#L33).

### 7. These shooting ideas do not require a Pedro 3 migration

Our TeamCode dependency is com.pedropathing:ftc:2.1.2. Silver Circuits declares com.pedropathing:revhub:3.0.1. Feed readiness, boost scheduling, hysteresis, and motor-output experiments can be evaluated in our existing architecture. This review does not establish a shooting-consistency benefit caused by Pedro 3. [Our dependencies](../../TeamCode/build.gradle), [Silver dependencies][vc-deps].

## Prioritized experiment plan

| Order | Experiment | What to record / decide |
| --- | --- | --- |
| 1 | Characterize the current shooter with 1-ball shots and repeat bursts at a fixed target | RPM before contact, minimum RPM, RPM before the next ball, recovery time, output, saturation, loaded voltage, loop dt, and per-ball make/miss |
| 2 | Add initial speed gating with a pending request, stable-ready condition, timeout, and cancellation | Does first-ball accuracy improve without unpredictable trigger delay? Start all feed-relative clocks at actual feed start. |
| 3 | Compare boost disabled, current timed floor, and a bounded recovery pulse | Does ball-2/later accuracy improve without overspeed between balls? Change pulse timing/duration separately from gains. |
| 4 | Compare normal slew with a bypass limited to the recovery pulse | Does requested power reach the motor early enough? If already saturated, diagnose energy/feed/recovery constraints. |
| 5 | Hold flywheel settings fixed and vary intake power during feed | Does spacing or loaded voltage improve while preserving required throughput? |
| 6 | Add event-specific compensation only if remaining error warrants it | Validate beam/RPM events against slow-motion video; distinguish confirmed contact from inferred timing. |

Use small repeated A/B blocks (for example 10 bursts per condition), fixed distance/aim/ball order, and more than one battery state. Label each ball's outcome; a burst make count alone can hide an accurate first ball and weak later balls. If ball types differ, record type and sequence rather than assuming one RPM/power profile fits all.

Our subsystem already exposes RPM, output, slew, voltage, and current telemetry; the 2026 tuner does not currently provide the legacy CSV pipeline. Capture a synchronized trace if possible. Control-code changes and new logging are future implementation work, not changes made by this review.

## Local baseline fingerprints

SHA-256 of raw local file bytes at review time. These identify the uncommitted inputs; they do not archive their contents.

| File | SHA-256 |
| --- | --- |
| ShooterSubsystem2026.java | 07EF63195B8D3C94CFB762B9B415E63C71573A1274E2A05573AE7A282B11EB3C |
| IntakeWithSensorsSubsystem2026.java | BBDD9C450C2267AD0FEADBE66E539B455059E984DEDB0DC156C5428EA29D6FDF |
| Pickles2026Teleop.java | 0B6C88CC7A4A3D54B936D97A27A652F6264C0B92F7A9F386F6A09CAC5DB327C9 |
| ShooterTuning2026.java | 7BB27759FBAD8F3A8D566BD5B2F45D4EEDFA883152B6A7AF663ADC51910B28BF |

Local source links track the working files and can move as edits continue. External source links below are pinned to the reviewed commits. This is the first comparison; there is no previous report to diff.

[ftc-launch]: https://github.com/FTC-23511/BioBuzz-2026/blob/e2e2e86d1076717105ae16c76fde906a44ca1d38/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commandbase/commands/Launch.java#L35-L66
[ftc-launcher]: https://github.com/FTC-23511/BioBuzz-2026/blob/e2e2e86d1076717105ae16c76fde906a44ca1d38/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commandbase/subsystem/Launcher.java#L123-L140
[ftc-constants]: https://github.com/FTC-23511/BioBuzz-2026/blob/e2e2e86d1076717105ae16c76fde906a44ca1d38/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/globals/Constants.java#L65-L77
[ftc-tele]: https://github.com/FTC-23511/BioBuzz-2026/blob/e2e2e86d1076717105ae16c76fde906a44ca1d38/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/teleop/StarterBotTeleOp.java#L105-L124
[ftc-tuner]: https://github.com/FTC-23511/BioBuzz-2026/blob/e2e2e86d1076717105ae16c76fde906a44ca1d38/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/launcher/LauncherMotorTuner.java#L72-L78
[vc-tele]: https://github.com/VCSilverCircuits/BiobuzzV1-Pedro-3/blob/2ba1221a4c1edf5a948c30d5ee8d2d7c2c149429/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vcsc/opmodes/teleOp/scrimTeleOp.java#L31-L104
[vc-auto]: https://github.com/VCSilverCircuits/BiobuzzV1-Pedro-3/blob/2ba1221a4c1edf5a948c30d5ee8d2d7c2c149429/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vcsc/opmodes/auto/scrimAutoRed.java#L147-L204
[vc-outreach]: https://github.com/VCSilverCircuits/BiobuzzV1-Pedro-3/blob/2ba1221a4c1edf5a948c30d5ee8d2d7c2c149429/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vcsc/opmodes/teleOp/outreachTeleOpCub.java#L31-L103
[vc-deps]: https://github.com/VCSilverCircuits/BiobuzzV1-Pedro-3/blob/2ba1221a4c1edf5a948c30d5ee8d2d7c2c149429/build.dependencies.gradle#L19
[haz-launcher]: https://github.com/13201Hazmat/26-27.Biobuzz/blob/9166ce0088bd25a9e0c9174b9356e078f937b592/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Launcher.java#L20-L139
[haz-calc]: https://github.com/13201Hazmat/26-27.Biobuzz/blob/9166ce0088bd25a9e0c9174b9356e078f937b592/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/data/Calc.java#L17-L29
[haz-test]: https://github.com/13201Hazmat/26-27.Biobuzz/blob/9166ce0088bd25a9e0c9174b9356e078f937b592/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testop/LauncherTest.java#L25-L34
[haz-transfer]: https://github.com/13201Hazmat/26-27.Biobuzz/blob/fd83188da19c28bce9480c59305f8364a59552f3/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/transfer/Transfer.java#L81-L138

