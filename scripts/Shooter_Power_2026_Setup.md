# Shooter power configuration checkpoint

These controls are shared by the 2026 autonomous programs, teleop and shooter
tuning through `ShooterSubsystem2026` and `ShotSequenceController2026`.
They do not change shooting RPM selection, feeding permissions, intake speeds,
ball counting, motor FLOAT behavior or shot timing by default.

## Baseline defaults

The defaults intentionally reproduce the meet controller, rather than the model's
candidate settings. Select the relevant class in Panels. Do not change settings
until the unchanged-behavior checkpoint has been checked on the robot and committed.

| Class | Setting | Default / meaning |
| --- | --- | --- |
| ShooterSubsystem2026 | `SLEW_PER_SECOND` | `1.5`, used in both directions by default |
| ShooterSubsystem2026 | `ENABLE_ASYMMETRIC_SLEW` | `false`; enable to use the running rates below |
| ShooterSubsystem2026 | `RUNNING_SLEW_UP_PER_SECOND` | `1.5` |
| ShooterSubsystem2026 | `RUNNING_SLEW_DOWN_PER_SECOND` | `1.5` |
| ShooterSubsystem2026 | `ENABLE_STARTUP_PROFILE` | `false` |
| ShooterSubsystem2026 | `STARTUP_SLEW_UP_PER_SECOND` | `1.5`; used only during enabled startup profile |
| ShooterSubsystem2026 | `STARTUP_SLEW_DOWN_PER_SECOND` | `1.5`; used only during enabled startup profile |
| ShooterSubsystem2026 | `STARTUP_ENTRY_MAX_RPM` | `200`; fresh measured RPM must be below this at enable |
| ShooterSubsystem2026 | `STARTUP_EXIT_TOLERANCE_RPM` | `50` |
| ShooterSubsystem2026 | `STARTUP_EXIT_MAX_SLOPE_RPM_PER_SECOND` | `200` absolute measured RPM change per second |
| ShooterSubsystem2026 | `STARTUP_EXIT_STABLE_SECONDS` | `0.15`; tolerance and slope must hold continuously |
| ShooterSubsystem2026 | `STARTUP_MAX_SECONDS` | `2.0`; bounded handoff, even if RPM is never reached |
| ShooterSubsystem2026 | `VOLTAGE_COMP_MIN_GAIN` | Existing `1.0`, so feedforward is not reduced at high voltage yet |
| ShooterSubsystem2026 | `VOLTAGE_COMP_MAX_GAIN` | Existing `1.05` |
| ShotSequenceController2026 | `ENABLE_TIMED_BOOST` | Existing `true` |
| ShotSequenceController2026 | `FEED_SECONDS` | Existing `1.2` |
| ShotSequenceController2026 | `BOOST_DELAY_SECONDS` | Existing `0.18` after feed begins |
| ShotSequenceController2026 | `BOOST_DURATION_SECONDS` | Existing `0.50` |
| ShotSequenceController2026 | `BOOST_MINIMUM_POWER` | Existing `0.75`; controls the flat floor when ramping is off |
| ShotSequenceController2026 | `BOOST_BYPASSES_SLEW` | Existing `true`, bypasses upward and downward limits while boost is active |
| ShotSequenceController2026 | `ENABLE_RAMPED_BOOST` | `false`; enable to replace the flat floor with the endpoints below |
| ShotSequenceController2026 | `BOOST_START_POWER` | `0.75` |
| ShotSequenceController2026 | `BOOST_END_POWER` | `0.75`; linearly interpolated over the boost window |
| ShotSequenceController2026 | `ENABLE_BOOST_VOLTAGE_SCALING` | `false` |
| ShotSequenceController2026 | `BOOST_NOMINAL_V` | `12.5`; reference for the boost floor when voltage scaling is enabled |
| ShotSequenceController2026 | `BOOST_OVERSPEED_TAPER_RPM` | `0`, disabled; positive values taper extra boost above target |

Slew values are power units/second, not RPM/second. A rate of 1.5 allows at most
0.15 power change in 100 ms. This limits duty-cycle changes, not measured current.
No new current, RPM-ready or chassis-speed gate is introduced.

## Startup behavior when enabled later

Startup is considered once per disabled-to-enabled transition. Eligibility uses
the fresh RPM from the controller loop, including when a motor has been stopped
in software but is still coasting. Ordinary `spinUp(...)` calls that update the
target do not restart the profile. Neither do RPM dips while feeding.

Handoff occurs after the stable interval, on timeout, when feeding begins, or if
the startup profile is disabled live. A feed already requested before the first
controller update suppresses startup mode. Handoff retains the actual previous
power for slew limiting and never waits to grant permission to feed. Stop still
commands zero immediately. Selecting startup limits does not require enabling
asymmetric running limits; normal running can retain `SLEW_PER_SECOND`.

Shooter rates and feedforward settings apply on subsequent controller loops.
The startup-enable choice can arm only at a new enable transition; turning it on
mid-run does not rearm startup. Turning it off ends an active startup profile.
The startup elapsed time freezes at handoff and resets on motor stop/disable.

## Boost behavior when enabled later

Sequence timing, ramp endpoints, voltage-scaling choice/reference, taper and slew
bypass are snapshotted at the start of each volley. Panels edits apply to the next
volley, so the program cannot accidentally change the profile halfway through it.

The unscaled floor is either `BOOST_MINIMUM_POWER` or a linear start-to-end ramp.
Optional scaling multiplies it by `BOOST_NOMINAL_V / filtered_battery_voltage` and
clips it to 0–1. It is independent of the feedforward gain cap. Above target RPM,
a positive taper width reduces the extra floor linearly to zero by
`target + BOOST_OVERSPEED_TAPER_RPM`. Normal PIDF remains active throughout.
The floor is a minimum TOTAL output, not power added on top of PIDF.

Shaping uses the shooter's voltage/RPM samples inside the controller update.
The existing scheduling between the shot request and controller update is retained.
Invalid new configuration values produce an error during Init; validate again at
enable/sequence start to catch invalid edits made after Init.

## Telemetry, logging and checkpoint verification

Existing logging storage defaults remain unchanged: memory until Stop, protected
through 35 s auto / 125 s teleop, with the same optional practice checkpoints.
The new fields reuse cached controller values; no new hardware reads are added.
They increase the size of each stored detailed row.

Basic logs now contain `shooter_power_profile`, `shooter_startup_elapsed_s`, and
`shooter_startup_exit_reason`, with `SHOOTER_PROFILE` transition events. `RUNNING`
with reason `PROFILE_DISABLED` is the expected legacy-default state while enabled.
Detailed logs include configured and active slew rates, measured RPM slope,
active volley profile parameters, pending requested floor, and the floor consumed
by the controller before scaling, after scaling, and after overspeed taper.
`controller_sample_boost_bypass` refers to that cached controller calculation.
Existing requested/actual/no-boost power diagnostics use the selected slew limits.

CSV schema version 1 and existing column names are retained with additive columns.
The converter recognizes the new state/reason strings and still reads older logs.
Driver Station logging telemetry includes the active profile, slew rates and floors.

Desktop checks compare 2,000 prescribed controller steps against the previous
default P+FF/voltage/boost/slew calculation, including varying dt, targets, RPM,
voltage, boosts and motor restarts. Opt-in profile handoff, flat/ramped boost,
sequence snapshots, saturation, log alignment and conversion have separate checks.
These establish calculation equivalence, not identical Control Hub loop timing.
Run a normal robot baseline check before treating this as the competition fallback.

Run the production compilation and the desktop shooter/logging/shot/intake/converter
checks from PowerShell with a JDK and Python on PATH:

```powershell
.\scripts\run_shooter_configuration_checks.ps1
```

It uses cached Gradle dependencies by default; add `-Online` if dependencies need
to be downloaded. Temporary classpaths and test classes stay under `build/`.

Automated comparison opmodes and candidate parameter changes are separate later steps.
