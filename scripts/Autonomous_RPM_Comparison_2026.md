# Autonomous RPM comparison — November 6, 2026

The three qualifier opmodes share fixed-RPM defaults from `AutonomousShooterRpm2026`:

* Close preload Low RPM: **3050**, in both `CloseShootingAlone` and `CloseWithPartner`.
* High RPM: **3150**, for all later shots in both close programs and every fixed-RPM shot
  in `FarWithPartner`, including its delayed first volley.

Each opmode has `USE_DISTANCE_BASED_RPM = false` by default. Both close opmodes also have
`USE_FIRST_SHOT_LOW_RPM_OVERRIDE = true` by default. Select the flags in source or Panels
before Play. The override setting is captured when the first volley starts.

| Distance regression | Close first-shot override | Close preload volley | Later volleys |
|---|---|---|---|
| false | either | 3050 RPM | 3150 RPM |
| true | true (default) | 3050 RPM | TeleOp regression |
| true | false | TeleOp regression | TeleOp regression |

Far With Partner has no Low override: its first and subsequent volleys use 3150 in fixed
mode, or regression when enabled. Its partner delay remains unchanged.

The close preload override covers spin-up, movement to the preload position, and the entire
first feed sequence. Continuous regression updates cannot replace Low RPM during that volley.
It is released only after the first feed sequence completes; later shots from a close location
do not reapply it. Normal later-phase commands also clear any remaining preload override.

Regression uses the shared TeleOp model and live configuration:

```text
target X = 58
target Y = 60 when robot Y <= 72; otherwise 84
distance = hypot(target X - robot X, target Y - robot Y)
flywheel RPM = 16.2 * distance + 2410
```

These are flywheel RPM values. Autonomous uses the equation and target selection without
importing the TeleOp shot table or GP2 RPM trim. Future TeleOp equation/target edits are shared.

## Comparison at planned shooting waypoints

The last column is the selected RPM with regression enabled and the default close-preload
override enabled. The raw-regression column shows what disabling that override would produce.

| Program | Shot | Planned robot X, Y (in) | Distance (in) | Fixed RPM | Raw regression RPM | Raw change | Regression + preload override |
|---|---|---:|---:|---:|---:|---:|---:|
| Close alone | Preloads | 54.1780, 11.0000 | 49.15 | 3050 | 3206.2 | +156.2 (+5.12%) | 3050.0 |
| Close alone | Far flower shots / optional far garden shot | 35.0000, 120.0000 | 42.72 | 3150 | 3102.1 | -47.9 (-1.52%) | 3102.1 |
| Close alone | Garden balls from close (default) | 42.9240, 14.3419 | 48.08 | 3150 | 3188.9 | +38.9 (+1.24%) | 3188.9 |
| Far with partner | Far preloads + far flower | 35.0000, 120.0000 | 42.72 | 3150 | 3102.1 | -47.9 (-1.52%) | 3102.1 |
| Far with partner | Extended plan: garden + optional near-flower shot | 42.9240, 14.3419 | 48.08 | 3150 | 3188.9 | +38.9 (+1.24%) | 3188.9 |
| Close with partner | Preloads | 54.1780, 11.0000 | 49.15 | 3050 | 3206.2 | +156.2 (+5.12%) | 3050.0 |
| Close with partner | Help-far branch: far flower | 35.0000, 120.0000 | 42.72 | 3150 | 3102.1 | -47.9 (-1.52%) | 3102.1 |
| Close with partner | Garden + optional near-flower shot | 42.9240, 14.3419 | 48.08 | 3150 | 3188.9 | +38.9 (+1.24%) | 3188.9 |

Repeated shots at the same waypoint are grouped. Coordinates come from current opmode source
and fixed values come from compiled defaults. Raw regression is about 3206.2 at the close
preload point, 3102.1 at the far shooting point, and 3188.9 at the later close point. With the
preload override on, the first value is held at 3050. Compared with the standardized High
setting, regression lowers far shots by about 48 RPM and raises later close shots by about 39.

## Runtime behavior

* Existing start/change commands retain their fixed RPM as fallback for that phase.
* With regression disabled, fixed targets remain Low for close preloads and High afterward.
* With regression enabled and no active preload override, RPM follows the current Pedro pose
  each loop while the shooter is enabled, including travel between shots.
* Toggling back to fixed restores the last fixed RPM requested by that sequence.
* Invalid pose/equation values fall back to the current phase's fixed RPM.
* Updates never start the flywheel during Init or restart it after a parking/Stop command.
* Paths, chassis headings, partner delays, spin-up delays, feed windows, boost,
  voltage compensation, and shot-start conditions retain their existing behavior.
  No speed-ready wait was added.

The comparison uses planned waypoints. Actual commands use measured pose and can differ if a
path ends early; close preload paths already have an existing 1-second timeout. Telemetry
shows the selected RPM mode, including `FIRST VOLLEY LOW OVERRIDE`, fixed/regression values,
distance, and existing target/measured flywheel RPM.

Compilation passed with `:TeamCode:compileDebugJavaWithJavac --offline`. The **119 checks** in
`scripts/tests/AutonomousRegression2026Test.java` cover standardized defaults, all flag
combinations, repeated updates throughout the first-volley override, release after completion,
normal regression, mode switching, shared TeleOp settings, invalid-pose fallback, and stopped
shooter behavior. They do not validate trajectories on the robot. The test's `--report` option
regenerates the numeric table above.
