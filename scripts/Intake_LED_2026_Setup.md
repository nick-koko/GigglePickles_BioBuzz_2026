# Intake controls and driver LEDs

Automatic jam recovery has been removed. Hold B to reverse both intake motors
manually; release B to return to the selected intake mode. X stops the intake.

## A buttons and controller 2 capacity override

- Tap A on either controller to select normal forward collection.
- While controller 2 A is held, the intake motor may collect even when the
  tracked count is four. The transfer still respects its normal upper-storage
  capacity stop. No motor speeds or shooting-feed powers have changed.
- Release controller 2 A to restore the intake capacity stop. The selected
  collection mode remains forward; an under-capacity intake can continue normally.
  Release does not start an intake that X has stopped.
- X on either controller cancels shooting and stops both collection motors, even
  if A is held. Continuing to hold A does not restart a stopped intake; a fresh A
  press can start it again. Existing flower-down automatic collection is retained.
- B reverse and intentional shooting feed retain their existing output priority.
  The held-A capacity override does not open the shooter gate or change feeding power.

## Sustained blocked-beam recovery

`IntakeWithSensorsSubsystem2026.ENABLE_SUSTAINED_BEAM_RECOVERY` defaults true.
`SUSTAINED_BEAM_CONFIRM_SECONDS` defaults to **0.10 seconds** and is adjustable in
Panels. Edge-based detection remains for fast normal counting. The recovery path
also considers a contiguous blocked-beam stack from the shooter toward the entrance.
Each included sensor must remain blocked for the confirmation interval. A lower
beam blip does not reset confirmation of upper positions.

Recovery only raises the latched occupancy to a larger confirmed stack; it never
counts one held beam repeatedly and never lowers the count because a hole clears
a beam. The same loop that recovers four applies the normal intake capacity stop,
unless controller 2 A is held. Transfer capacity still stops at its existing upper
two occupied positions. The shared subsystem applies this recovery in autonomous too.

Feeding, reverse and diagnostic motor tests suppress counting and reset the
confirmation timers. After feeding, a remaining stack must confirm again.
Disabling ongoing recovery retains edge counting and the older one-shot
post-reset/post-feed fallback, controlled by `STATIONARY_RESYNC_SECONDS` (0.25 s).
That fallback cannot shorten the selected ongoing confirmation delay while enabled.

A sensor stuck blocked can conservatively produce false occupancy; held controller
2 A provides intake-only recovery as requested. It does not override a transfer
capacity stop. A sensor stuck clear can still prevent a valid top-down pattern.

Telemetry shows the override, confirmed stack size and recovery-event count. Basic
logs include `INTAKE_OVERRIDE` and `BALL_COUNT_RECOVERY` events. Detailed logs include
the held request, capacity reached, confirmation setting/timers and the most recent
recovery's before/after count and beam mask. These reuse cached beam samples.

### Whole-match replay

`scripts/replay_match4_intake_2026.py` compares the compiled production subsystem
with the pre-recovery checkpoint against every Match 4 auto/teleop sample. Run the
desktop check script first, then supply `--log-root` and `--baseline-ref` (the commit
before sustained recovery). It requires Python/matplotlib and a JDK; it writes an
HTML report, plots and all per-loop results under `Data/league-20261119/match4-count-replay`.

It seeds only the first observed count, then replays the entire run without
resetting to historical counts between pickups. Baseline counts and motor commands
must reproduce the logs. This is a fixed-sensor replay: earlier new motor stops
would change later ball movement on a real robot. A reported new count of four is
not independent proof of four physical balls. Controlled one-ball and full-stack
robot tests remain necessary to check overlapping beam coverage and hole effects.

## Driver LEDs in Pickles 2026 Minimal Teleop

- Outside shooting range: solid red while aiming or requesting a shot. Set
  `SHOOT_RANGE_WARNING_ONLY_WHILE_AIMING_OR_FIRING` false to show it everywhere.
- In range and aligned: both outputs flash white together, twice per second.
  Alignment must remain true for 0.15 seconds before flashing starts.
- Otherwise: the existing pollen count colors remain available.

White indicates alignment and range. Flywheel RPM and pollen count are not part
of this indication. Chassis aiming uses the configured shooter heading offset,
within 2 degrees and at most 10 degrees/second of yaw. Tracking-turret mode uses
the turret's reachable/settled measured aim. Red overrides white and stage pulses.

White is selected at 1925 microseconds, inside Prism's solid-color range.
Sources: [Prism PWM guide](https://www.gobilda.com/content/user_manuals/3118-2855-0001_user-guide.pdf)
and [goBILDA PWM color chart](https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1).

## Robot verification

Check manual B reverse/release, X stop, and capacity stops on the robot. Check
tap/hold/release controller 2 A at full capacity, X while A remains held, closely
spaced pickups, and a small clear blip after a full stack has been counted. Check
red/white visibility across the field and verify the white cue against the
physical shooter direction. Software checks use simulated hardware; the LED
alignment settings still need validation on the robot.
