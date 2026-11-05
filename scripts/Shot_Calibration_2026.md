# Simple 2026 shot calibration

`Pickles2026Teleop.USE_SHOT_TABLE` selects the RPM/aim source:

* `false` (default): regression RPM = `27.0 * distanceInches + 2300.7`, with zero
  table aim offsets. Slope and intercept are editable in teleop configuration.
* `true`: look up RPM and aim offsets from `ShotCalibrationTable2026.SAMPLES`.
  An empty table or a table with no valid rows falls back to regression and
  reports that fallback in telemetry.

The table is intentionally empty until measured 2026 shots are entered. Last
year's robot, hood settings, field targets, and RPM values were not imported.

## Enter rows

Edit `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/shot/ShotCalibrationTable2026.java`:

```java
public static Sample[] SAMPLES = {
    //         robotX robotY flywheelRPM deltaAimX deltaAimY
    new Sample(54.0,  12.0,  3200.0,     0.5,      -1.0),
};
```

This is a format example, not a measured/calibrated shot. Replace it with actual
measurements. Each row has exactly those five values; there is no hood, zone,
target ID, or row number to maintain. RPM is **flywheel** RPM, not motor RPM.
Positions and aim deltas use field inches in the current Pedro coordinate frame:
positive X moves right and positive Y moves up-field.

Teleop first chooses its existing nominal shooting target using the robot's Y
position, then adds the table deltas:

```text
aim X = selected target X + deltaAimX
aim Y = selected target Y + deltaAimY
```

For example, the same delta `(0.5, -1.0)` moves `(58, 60)` to `(58.5, 59)` and
`(58, 84)` to `(58.5, 83)`. The offsets drive either chassis aiming or turret
aiming, depending on turret mode. These are field-coordinate offsets, not a hood
or elevation adjustment. The existing target split remains; rows need no zone
labels. Calibrate with the target that this existing selection chooses at each
robot location.

## Lookup behavior

The nearest measured row is used directly within 0.5 inches of it. Otherwise the
nearest three rows are blended using inverse-square distance weights, including
RPM and both offsets. With fewer than three rows, all available rows are used.
`NEAREST_SAMPLE_COUNT` and `EXACT_SNAP_DISTANCE_INCHES` are configurable.
Null rows, non-finite numbers, and nonpositive RPM rows are ignored.

A single row returns its settings everywhere; sparse or far-away rows are not
evidence of a calibrated shot at the current position. Telemetry shows how many
rows contributed and the distance to the nearest row. The lookup does not invent
an extrapolated RPM above or below its contributing rows.

Regression distance and the red range-warning LEDs use distance to the **nominal
physical shooting target**, without aim offsets. Aim offsets therefore do not
alter range classification or feed back into regression RPM.

## RPM controls and calibration

The shooter continues to start at Play as before. While enabled, its RPM now
continuously follows the selected regression/table calculation as the robot
moves, even after LB is released. LB can start it when
`AUTO_AIM_STARTS_SHOOTER` is enabled; GP1 RB still toggles it.

GP2 D-pad up/down adjusts `SHOOTER_RPM_TRIM` by 25 RPM. This trim is added once
after either calculation. It starts at zero and is editable in Panels. It is
the replacement for adjusting the old fixed `SHOOTER_TARGET_RPM`.

To collect a row, park at a known X/Y, tune the RPM trim until shots work, and
record the actual target flywheel RPM shown in telemetry. Enter the measured
aim deltas with that row. Clear the global RPM trim when validating a row whose
RPM already includes your tuning adjustment, so that adjustment is not counted
twice.

To temporarily reproduce fixed-RPM practice, leave the table disabled, set
regression slope to zero, and set its intercept to the desired RPM. The normal
regression defaults remain slope 27.0 and intercept 2300.7.

`SHOT_TUNING_TELEMETRY_ONLY` changes only telemetry layout. Both layouts show the
RPM source, base RPM, trim, nominal target, aim deltas, and actual shooter target.
`SHOOTER_FALLBACK_RPM` is used if the selected RPM calculation is invalid; it
does not override a valid regression/table result.

## Fixed overwrite bug

Previously, LB called `spinUp(regressionRpm)` and the same update later called
`spinUp(SHOOTER_TARGET_RPM)`, overwriting it before the shooter controller's next
update. The normal target therefore remained the fixed RPM. The updated loop
has one final shooter command using the selected calculation. At 30 inches,
zero trim now commands 3110.7 RPM; at 60 inches it commands 3920.7 RPM.

This changes the actual flywheel target compared with that previous behavior.
The original regression coefficients are preserved, rather than assumed to be
validated by runs that were actually using a fixed target.

Autonomous shot RPMs remain the explicit values in their existing autonomous
programs. This table is integrated into 2026 Minimal Teleop only.

## Verification

Compile with `./gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline`.
`scripts/tests/ShotCalibration2026Test.java` runs against the compiled production
classes and Gradle compile classpath without initializing robot hardware. It
checks table interpolation, missing/invalid data, both nominal targets and aim
deltas, regression fallback, trim, and the actual flywheel target stored by
`ShooterSubsystem2026` after the teleop command function runs.
