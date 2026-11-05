# 2026 turret setup and startup

Configuration lives in `TurretSubsystem2026`. The old 2025 turret and teleop are
unchanged. No hood servo or old shot tables are used. Existing shooter RPM and
chassis P/D/Pedro settings are retained.

## Choose a mode

Set `MODE` before pressing Init (in source or Panels). Changing mode, geometry,
encoder signs, zero calibration, or preload requires Stop and Init again.

| Mode | Hardware required | Left bumper | Turret behavior |
| --- | --- | --- | --- |
| `CHASSIS_ONLY` (default) | No turret hardware | Existing chassis turn toward the goal, with full translation control | No turret commands |
| `FIXED` | Both turret servos | Chassis aims accounting for the fixed turret bearing | Servos hold `FIXED_ANGLE_DEGREES`; no encoder reads |
| `TRACKING` | Both servos, quadrature and analog encoder | Turret aims; chassis translation and right-stick rotation remain driver controlled | Holds the last requested turret angle when LB is released |

The existing GP2 right-bumper aim request follows the same selection. No buttons
were repurposed. `CHASSIS_ONLY` also suits a turret that is mechanically locked
or disconnected. For a mechanically locked shooter pointing away from robot
front, set `CHASSIS_ONLY_SHOOTER_HEADING_DEGREES` to its robot-relative bearing.

In `FIXED`, the servos' own position control holds the mechanism. Encoder failures
do not prevent using this mode. Its position is open loop from the robot code's
perspective: check the actual shooter bearing and adjust
`SERVO_ZERO_OFFSET_DEGREES` as needed. A tracking calibration's learned servo
offset is shown in telemetry; it is not automatically copied into FIXED mode.

`FIXED_ANGLE_DEGREES`, rate limit, outer-loop P, and maximum trim can change live.
The fixed bearing used by chassis aiming includes the configured front/zero
orientation. Changing the fixed angle temporarily blocks feed while it settles.

## Startup and autonomous handoff

* Teleop Init only maps turret hardware and reads sensors. It never sends turret
  positions, changes servo direction/PWM, or resets the quadrature counter.
* With a valid autonomous reference, passive encoder reads include movement after
  autonomous Stop and throughout Init. Play resumes control without centering.
* With no valid reference (practice, controller restart, or rejected handoff),
  Play commands the known center with the opposing preload, waits for settling,
  and establishes the encoder reference. Driving remains available. Turret aim
  and shooting are blocked until calibration succeeds.
* RT held during calibration must be released and pressed again afterward.
* In FIXED mode, Play commands the fixed position and allows a settling delay
  before shooting. CHASSIS_ONLY preserves the existing shooting behavior.
* Stop never recenters the turret or toggles its PWM.

This relies on the team's procedure of pressing Play only when TELEOP begins.

The five current NextFTC autonomous programs (`BlueSideTest`,
`BlueSideFrontOfHive`, `RedSideFrontOfHive`, `AutonomousTesting`, and
`leavandpark`) calibrate during autonomous Init, wait for turret startup before
running their sequence, and save a fresh reference at Stop after Play. Their
paths and shot sequences are unchanged. TRACKING holds
`AUTONOMOUS_HOLD_ANGLE_DEGREES` in autonomous, since these paths already rotate
the chassis for shooting; it does not track the teleop goal coordinates in auto.
The emergency direct-drive autonomous deliberately clears the turret handoff and
does not initialize turret hardware; teleop then recovers after Play.

The handoff includes the quadrature offset, measured angle/current count, analog
phase, servo calibration, and both last servo commands. Servo zero is never
relearned from coasting movement during a handoff. Configuration changes, an
out-of-range position, or disagreement between quadrature movement and analog
phase reject the reference. The analog comparison helps detect count resets;
its repeating phase cannot independently prove counter continuity in every case.

## Mechanical and encoder settings

* Servo-to-turret ratio remains 1:1, so `SERVO_TRAVEL_DEGREES = 355` stays unchanged.
* Encoder ratio is `81.0 / 45.0 = 1.8`: 4096 shaft counts gives 7372.8 counts per
  turret revolution, or 20.48 counts per turret degree. This is derived at Init.
* `SERVO_DIFFERENTIAL_DEGREES = 2` is **total separation**: left +1 degree,
  right -1 degree. It applies during centering, tracking, and fixed holding.
  Individual commands are clipped at the configured travel limits, so separation
  decreases at an endpoint, matching the old behavior.
* For a front-facing turret zero, use
  `ROBOT_FRONT_TO_TURRET_ZERO_OFFSET_DEGREES = 0`. For a rear-facing zero, use
  `180`. `ROBOT_FRONT_RELATIVE_SIGN` maps field/robot angles into servo-angle sign.
* Verify `QUAD_DIRECTION_SIGN` and `ABSOLUTE_DIRECTION_SIGN`: both reconstructed
  angles must increase when the commanded turret angle increases.
* Encoder hardware defaults match the old robot: `back_left` for quadrature,
  `analog_turret_encoder` for analog, and `Lurret`/`Rurret` for servos. This code
  only reads the shared motor's encoder; it never changes that motor's mode,
  power, or direction. Current Pedro 2.1.2 Mecanum construction does not reset it.

Absolute phase repeats every 200 turret degrees, so cold calibration intentionally
centers first, then considers only physical positions inside the turret limits
near the known center. It does not guess an arbitrary initial angle.

Set the analog zero for the actual assembled gears before using TRACKING:

1. Use FIXED mode at zero to establish/check the intended mechanical center.
2. Read the analog phase at that physical reference. Tracking Init displays phase
   without commanding movement.
3. If that physical reference is turret zero, use
   `ABSOLUTE_TURRET_OFFSET_DEGREES = -ABSOLUTE_DIRECTION_SIGN * phase / 1.8`.
   Use the actual ratio if changed. Re-Init after editing the calibration.
4. Verify center and a small positive/negative angle before testing large moves.

Keep autonomous's held turret bearing consistent with the shooting direction
assumed by the existing chassis headings. A rear-facing zero is not automatically
compensated by rewriting autonomous paths.

Telemetry reports mode, startup state, handoff source, analog phase, measured and
target angle, servo offset/positions, target reachability, settling, and feed
permission. Calibration defaults to a 2-second settle and a 6-second timeout.
It faults if the centered analog estimate differs by more than 20 degrees from
the expected center. Correct phase/sign/mechanical setup rather than accepting
an unknown zero. A fault requires correcting the issue and restarting the opmode.

Target reachability and settled-angle telemetry are informational. Once startup
is valid, RT remains driver controlled; it does not wait for every aimed shot to
meet an angle/speed tolerance. Fixed-mode settling is time based because it has
no encoder requirement.

## Verification

Compile: `./gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline`.

`scripts/tests/TurretController2026Test.java` runs on a desktop JDK with only the
production `TurretController2026.java`: no Android runtime or robot is needed.
It covers passive Init, missing/invalid handoffs, coasting, counter changes during
Init, fixed holding without sensors, preload, geometry, limits, and calibration
faults. `TurretShotSequence2026Test.java` runs against compiled production classes
and the Gradle compile classpath, without initializing hardware, to verify the
feed gate and release/repress behavior during recovery.

On the robot, verify these transitions before relying on the turret in a match:
default CHASSIS_ONLY before installation; FIXED hold with chassis LB aim;
standalone TRACKING recovery after Play; autonomous-to-teleop transition with
no Init movement; and transition while the turret is still settling. Software
tests do not verify encoder wiring, servo programming, actual gear phase, or
hub/SDK output behavior on the assembled robot.
