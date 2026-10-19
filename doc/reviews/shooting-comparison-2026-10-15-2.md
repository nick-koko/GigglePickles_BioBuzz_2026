# Shooting comparison, round 2 — October 15, 2026

[Editable repository list](../team-code-comparison.md) · [Original three-team review](shooting-comparison-2026-10-15.md)

## Direct answer

**RevAmped contains a useful explicit recovery strategy:** when filtered flywheel speed sags, its controller switches from normal feedforward/feedback to full power, then returns to normal control as speed recovers. It also schedules a hood adjustment shortly after feeding starts. These are the strongest new ideas in this review. Neither relies on counting “ball 1.” [Recovery controller][rev-controller], [shoot command][rev-robot], [hood settings][rev-constants].

**Andover adds a useful stable-speed requirement before feeding**, along with different near/far transfer powers. BrickBot and ICE Penguin contain no shooting implementation in the inspected branch tips. [Andover ready check][ando-outtake], [feed sequence][ando-auto], [BrickBot hardware][brick-hardware], [Penguin hardware][penguin-robot].

No explicit ball-indexed “ball 2 gets more shooter power/RPM than ball 1” schedule was found. RevAmped's sag-dependent recovery can nevertheless address exactly the later-ball weakness we are interested in.

## Scope and comparison table

Reviewed the four newly added repositories, fetched all their advertised branch tips, and inspected the current local shooter path. The original three repositories were not refreshed in this round; their earlier report remains a separate snapshot. This is static code analysis, not measured evidence of accuracy or competition performance. External code was read, not built or executed.

| Repository | Shooting code found? | Power/recovery behavior | Feeding and other useful ideas | Relevance to our robot |
| --- | --- | --- | --- | --- |
| BrickBot15996/quickstart | No | Generic motor/controller framework only | Example drive and intake, recording/playback utilities | No shooter behavior to compare yet |
| FTC11329/11329-ICE-Penguin | No | Drive/intake and replay code; no flywheel controller found | Basic intake hardware | No shooter behavior to compare yet |
| RevAmped-Decode-V2 | Yes | Explicit SPINUP, REGEN, HOLD, SPINDOWN modes; REGEN uses full power in master | TeleOp waits for HOLD; timed hood compensation; velocity-controlled feeder with a slower far setting | Best recovery reference; DECODE mechanism and units differ |
| Andover 5273-Biobuzz | Yes | Two motors share software PID + velocity feedforward; no per-ball power step found | Speed must remain in range before timed feed; near/far transfer powers differ | Strong sequencing reference, with timeout and scheduler caveats |
| Our 2026 minimal TeleOp | Yes | Single motor, software PID/feedforward, normal slew limit | Trigger feeds immediately | Initial speed gating remains a gap |
| Our ShooterTuning2026 | Yes | Full-power floor begins 0.20 s after feed starts and lasts until the 3.0 s feed window ends | RPM-drop measurement is diagnostic | Recovery can be made responsive and bounded |

Our source observations are unchanged from the initial report: same four local file SHA-256 fingerprints. [Shooter controller](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem2026.java), [minimal TeleOp](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Pickles2026Teleop.java), [tuner](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/ShooterTuning2026.java), [intake/transfer](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeWithSensorsSubsystem2026.java).

## 1. RevAmped: recover from the actual speed dip

The robot's Flywheel class constructs FlywheelController and calls it during Robot.update(). This is connected behavior, not just an unused tuning class. It filters measured speed and chooses a control mode. [Flywheel integration][rev-flywheel], [robot loop][rev-loop].

| Starting state / condition in master | Result |
| --- | --- |
| HOLD, filtered speed below 92% of target but not below 75% | Enter REGEN |
| REGEN, positive speed error | Command full power: KICK_POWER defaults to 1.0 |
| REGEN, filtered speed above 95% of target | Return to HOLD |
| HOLD, filtered speed below 75% of target | Enter SPINUP, also using full power for positive error |
| HOLD, filtered speed above 105% of target | Enter SPINDOWN and request zero power |

The important idea is **restore power promptly when speed drops and release the recovery override when speed returns**. It does not wait a fixed time after feeding and does not need a reliable first-ball counter. The current REGEN output is full power; the additive REGEN_KICK formula in the same file is commented out. Older logging/worlds/redesign-family branches do use that additive recovery formula. [Current controller][rev-controller], [older controller][rev-old-controller].

For our 2026 subsystem, this suggests testing an automatic recovery floor using our existing minimum-power override API, with entry/exit thresholds, a maximum duration, and overspeed/cancel handling. Initially enable it only during a validated feed/burst after stable spin-up. Treat missed ball detection separately from the RPM evidence.

**Our existing controller already requests full power at small deficits.** At the minimal TeleOp's default 3855 RPM, its feedforward is approximately 0.607 and kP=0.02; about 20 RPM of deficit saturates the requested output. Normal slew limiting can still delay the applied output. The practical experiment is therefore to apply recovery promptly and stop the override appropriately, while recording both requested and applied power. Extra gain or a multiplier cannot help if applied power is already 1.0. [Our output calculation](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem2026.java#L192).

Do not copy RevAmped's state machine unchanged. Its SPINUP transition thresholds overlap between 70% and 75%, and its HOLD/SPINDOWN conditions overlap between 105% and 108%; at a fixed speed in these bands, the respective states can alternate on successive updates. This is a static inference from the comparisons, not an observed robot failure. Use correctly ordered thresholds in our implementation. Their velocity is expressed as wheel surface speed in inches/second, not our flywheel RPM. [Transitions][rev-controller], [unit conversion][rev-flywheel].

The MTI TeleOp fire binding waits for controller mode HOLD, then starts the shooting transition. Its separate canShoot() helper is not called elsewhere in master. A readiness helper's existence alone therefore does not establish the firing condition. [Actual fire binding][rev-tele].

## 2. RevAmped: a timed hood change during the burst

The close-shot command runs feed and hood compensation in parallel. When hood tracking is enabled, compensation waits **70 ms** from command start and applies a hood-position offset of **-25/255** through the tracking calculation. The nontracking path applies the offset immediately. The far compensation setting is zero in the reviewed defaults. Cleanup resets the tracking offset. [Command and cleanup][rev-robot], [compensation command][rev-hood-command], [constants][rev-constants], [tracking application][rev-math].

This is a timed trajectory adjustment, not proof that ball 1 passed and not an RPM-based hood manifold. It could compensate for a repeatable difference within a burst; the code alone cannot establish the intended physical cause or benefit.

Our current 2026 path has no hood controller. Keep this as a later experiment if an adjustable hood is present and RPM/feed measurements show a remaining trajectory error. The older local architecture has hood-related work, but bringing it into the new single-motor path would require a separate hardware-aware change.

RevAmped also uses a velocity-controlled feeder: shooting targets are **2300 encoder ticks/s close and 1200 ticks/s far** in its constants, plus reduced intake power for far shooting. This offers a concrete example of controlling feed cadence independently from flywheel speed. These units and signs are mechanism-specific. [Feeder controller][rev-feeder], [feed settings][rev-feed-constants].

### Features that are present but not integrated

The newest rew branch and MTI-StraightLine contain LQR controller code. Searching for its construction found no robot-path caller; Flywheel still constructs the hybrid FlywheelController. The file named FlywheelLQRTuner performs an open-loop voltage/velocity/acceleration sweep rather than running the LQR controller. Do not attribute LQR-based shooting performance to this robot. [LQR wrapper][rev-lqr], [tuner][rev-lqr-tuner], [actual flywheel][rev-flywheel].

## 3. Andover: require stable speed, then run a timed feed

Andover's Outtake.inRange() tracks continuous time within an RPM tolerance. Its callers use **±150 RPM for 75 ms in TeleOp and 100 ms in autonomous**. Once the wait completes, they start intake/transfer, open the gate, and run a fixed feed window: **1.0 s TeleOp, 0.85 s auto**. There is no per-ball RPM increase or recovery pause in that sequence. [TeleOp][ando-tele], [autonomous][ando-auto], [ready implementation][ando-outtake].

This directly strengthens the initial-speed recommendation from the first report: a stable interval is more useful than accepting a single in-range sample. Start our feed timer and boost/recovery timing at actual feed start, after the wait.

There are two important implementation qualifications:

- **Timeout also returns ready.** After 2750 ms, inRange() returns true even when RPM remains wrong. In our adaptation, distinguish READY from TIMEOUT and cancel or explicitly flag a failed spin-up. Changing the target by more than 25 RPM resets their timers.
- **The TeleOp wrapper can evaluate the same shooting action twice per scheduler update.** It includes the SequentialAction directly in a ParallelAction and also calls that same object's run() from a second child. The autonomous version explicitly avoids this duplication. Reuse the readiness/sequence concept with one owner of the action state. This caveat is based on source structure; no robot reproduction was performed.

Sources: [timeout and reset logic][ando-outtake], [TeleOp wrapper][ando-tele], [auto wrapper and comment][ando-auto].

Andover sets transfer power to **-0.7 close and -0.6 far**, with a conventional PID plus velocity feedforward controlling two shooter motors. Near/far feed speed deserves an A/B experiment on our mechanism; it is not compensation tied to which ball is being fired. Their storage current sensing detects accumulated intake/transfer load, not individual flywheel contacts. [Storage][ando-storage], [power loop][ando-power].

## 4. BrickBot and ICE Penguin: no shooting implementation found

BrickBot's only advertised branch is master. Its example robot registers intake and mecanum drive; no shooting subsystem or firing sequence was found across TeamCode Java/Kotlin sources. A flywheel reference in framework documentation is a recording example, not implemented shooter behavior. [Hardware][brick-hardware], [framework comment][brick-comment].

ICE Penguin also advertises only master. Its TeamCode tree contains drivetrain, intake, TeleOp/replay utilities, and no shooting controller or feed-to-shooter sequence found. Mark both as reviewed with no applicable shooter code in this snapshot; keep them enabled for future changes. [Robot][penguin-robot], [intake][penguin-intake].

## Recommended experiments, in order

| Priority | Experiment | Evidence to collect |
| --- | --- | --- |
| 1 | Initial feed gate requiring fresh RPM samples and a short stable interval; finite timeout with explicit failure | First-ball accuracy, trigger-to-feed delay, false-ready events |
| 2 | Compare current timed full-power floor with sag-triggered recovery that ends on recovery/overspeed or timeout | Raw/filtered RPM, requested/applied power, recovery duration, per-ball accuracy and saturation |
| 3 | Compare normal slew with bypass restricted to the recovery interval | Whether power actually reaches the motor before the next ball; overshoot between balls |
| 4 | Sweep transfer/intake feed settings at fixed flywheel targets, including separate near/far settings | Ball spacing, throughput, stalls, RPM before each ball, loaded voltage |
| 5 | Consider small timed hood trim only if RPM/feed tuning leaves repeatable trajectory error | Per-ball flight and hit location; measured hood response |

Use repeated A/B blocks at fixed distance and aim, recording ball order/type, loaded battery voltage, and loop timing. Verify inferred shot events against slow-motion video. These are proposed experiments, not robot-code changes made by this review.

## Branch and commit inventory

All 15 advertised branch tips across the four new repositories were screened. Detailed tracing emphasized defaults and the relevant variants; older history and unrelated source files were outside scope.

| Repository | Branch | Exact commit | Coverage |
| --- | --- | --- | --- |
| BrickBot | master (default) | 4692edfb880d151087cf2acddff02159e9396aa5 | Framework/example inventory; no shooter found |
| ICE Penguin | master (default) | 98d2a6ca0f2af0de4920b9c90f285d09ef1e85a4 | TeamCode inventory; no shooter found |
| RevAmped | master (default) | 3dbc4cb23f7a6302f3251291333a9daeeea8c2d9 | Detailed controller, TeleOp, robot, hood, feeder |
| RevAmped | rew (newest) | de8d9d6602f6f2297ca5dff48771441afb8d1553 | Same core hybrid controller; LQR integration check |
| RevAmped | MTI-StraightLine | a22b20e71704cfb81d1518dac59f2ba0ed747db4 | Same core Flywheel/Robot/controller as master; LQR additions |
| RevAmped | command-validator | ecdbfd63339ad6b6ec79fa628230541e7c6ee956 | Same core Flywheel/Robot/controller as master |
| RevAmped | MTI-StraightLine-FullSOTM | 0eac50bb1c137f9641adaceff60b130cc9f65569 | Same controller as master; branch screening |
| RevAmped | feature/MTI-TestCases | 97b593d1c8d510fe802d48b99b0e590e8032ed20 | Full-power recovery variant; older thresholds |
| RevAmped | 21 | 38e24bcf0cc3453fc04312c28a7d42aaa9f3ab6f | Older additive REGEN controller |
| RevAmped | redesign | cfcc9c028bbf0d12856e261308983710eb726384 | Same Flywheel/controller as 21 |
| RevAmped | worlds | 06092a130baba4f880d5579cb1263f723978424a | Older additive REGEN controller family |
| RevAmped | logging | 21927aa519235af7b6b9d22d7e5c5082d421ccf7 | Detailed older additive controller |
| RevAmped | logging-clean | 162a22fd16f4df356c085980bdabbc1a10bb6a94 | Same controller as logging; readiness logging |
| Andover | Quali-1 (default) | b86842b7e3436f47ad03ac206e9503a6bd982b33 | Detailed shooter, feed, TeleOp/auto call paths |
| Andover | autopaths | ff7e724e4616a363f44b1e9fd16dce8605019643 | Same Outtake; checked Storage/Bot differences |

Our branch remains **add-limelight-pollen**, HEAD **0ec7274e4ea55ba6f10ad3fc4cfb50cd498f41bb**, with uncommitted/untracked robot work. The four local 2026 source fingerprints match the [first report](shooting-comparison-2026-10-15.md#local-baseline-fingerprints); HEAD alone does not reproduce this baseline.

## What changed from the first report

The original three-team findings remain scoped to those repositories. This round adds concrete external examples of **speed-dependent recovery power and timed hood compensation**, and a **stable-speed dwell before feed**. Recovery that ends based on speed is now the most relevant new control experiment. The editable watchlist has been updated with discovered defaults and review status.

[rev-controller]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/shooter/FlywheelController.java#L28-L111
[rev-old-controller]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/21927aa519235af7b6b9d22d7e5c5082d421ccf7/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/shooter/FlywheelController.java#L28-L99
[rev-flywheel]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/shooter/Flywheel.java#L46-L145
[rev-loop]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java#L96-L108
[rev-robot]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java#L150-L201
[rev-hood-command]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java#L281-L309
[rev-constants]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotConstants.java#L37-L49
[rev-feed-constants]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotConstants.java#L64-L74
[rev-feeder]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/shooter/FeederWheel.java#L32-L69
[rev-tele]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MTITele.java#L136-L151
[rev-math]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/3dbc4cb23f7a6302f3251291333a9daeeea8c2d9/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/math/projectile/SimpleShooterMath.java#L198-L220
[rev-lqr]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/de8d9d6602f6f2297ca5dff48771441afb8d1553/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utils/control/FlywheelLQRController.java#L17-L49
[rev-lqr-tuner]: https://github.com/junkjunk123/RevAmped-Decode-V2/blob/de8d9d6602f6f2297ca5dff48771441afb8d1553/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/FlywheelLQRTuner.java#L41-L65
[ando-outtake]: https://github.com/Andover-Robotics/5273-Biobuzz/blob/b86842b7e3436f47ad03ac206e9503a6bd982b33/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Outtake.java#L216-L248
[ando-power]: https://github.com/Andover-Robotics/5273-Biobuzz/blob/b86842b7e3436f47ad03ac206e9503a6bd982b33/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Outtake.java#L140-L168
[ando-tele]: https://github.com/Andover-Robotics/5273-Biobuzz/blob/b86842b7e3436f47ad03ac206e9503a6bd982b33/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/Bot.java#L101-L128
[ando-auto]: https://github.com/Andover-Robotics/5273-Biobuzz/blob/b86842b7e3436f47ad03ac206e9503a6bd982b33/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/utils/BotActions.java#L94-L118
[ando-storage]: https://github.com/Andover-Robotics/5273-Biobuzz/blob/b86842b7e3436f47ad03ac206e9503a6bd982b33/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Storage.java#L17-L54
[brick-hardware]: https://github.com/BrickBot15996/quickstart/blob/4692edfb880d151087cf2acddff02159e9396aa5/TeamCode/src/main/java/org/brickbot/ftc/examplecode/RobotHardware.java#L10-L35
[brick-comment]: https://github.com/BrickBot15996/quickstart/blob/4692edfb880d151087cf2acddff02159e9396aa5/TeamCode/src/main/kotlin/brickbot/quickstart/opmode/BrickOpMode.kt#L55-L65
[penguin-robot]: https://github.com/FTC11329/11329-ICE-Penguin/blob/98d2a6ca0f2af0de4920b9c90f285d09ef1e85a4/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Robot.java#L9-L30
[penguin-intake]: https://github.com/FTC11329/11329-ICE-Penguin/blob/98d2a6ca0f2af0de4920b9c90f285d09ef1e85a4/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java#L15-L39

