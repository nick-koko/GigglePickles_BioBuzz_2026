# Pickles 2026 heading aiming trials

These settings are on `Pickles2026Teleop` in Panels. Hold GP1 LB to aim;
release it for manual rotation. Driver field-centric X/Y control stays available.

| Trial | `USE_PEDRO_AIM_PIDF` | P | D | Minimum | kS |
| --- | --- | --- | --- | --- | --- |
| Previous behavior / baseline | false | 0.0165 | 0 | 0 | 0.052 |
| Reduce P only | false | Reduce from 0.0165 | 0 | 0 | 0.052 |
| Custom PD | false | Tunable | Increase from 0 | 0 | 0 initially |
| Pedro autonomous heading PIDF | true | From Constants | From Constants | 0 | 0 initially |

- P means `AIM_TURN_KP_PER_DEGREE` (power per degree).
- D means `AIM_TURN_KD_PER_DEGREE_PER_SECOND` (power per degree/second).
- Minimum means `AUTO_TURN_MIN_POWER`, now a floor rather than an addition.
- kS means `AUTO_TURN_KS`, the additive friction compensation.
- `AUTO_TURN_MAX_POWER` caps either controller's final turn command.

For the custom controller, P uses wrapped heading error and D uses the negative
measured Pinpoint yaw rate. P is zero inside `AUTO_TURN_TOLERANCE_DEGREES`.
With D enabled, braking remains available inside tolerance while yaw rate exceeds
`AUTO_TURN_SETTLE_SPEED_DEGREES_PER_SECOND` (default 5 deg/s). With D at zero,
the default custom controller retains the previous zero-output tolerance behavior.

Compare friction strategies separately at the same P, D, and maximum:

| Strategy | `AUTO_TURN_MIN_POWER` | `AUTO_TURN_KS` |
| --- | --- | --- |
| No compensation | 0 | 0 |
| Minimum floor | Start small; 0.052 reproduces the previous constant's magnitude | 0 |
| Additive kS | 0 | Start small; previous value was 0.052 |

When the controller commands a correction toward the target, the code adds kS,
then raises the magnitude to the minimum if necessary, then applies the maximum.
Both settings can be nonzero. Neither creates output from a zero controller
command, acts inside tolerance, nor increases a command braking away from the
remaining heading error. A maximum of zero disables all automatic turn output.

Pedro mode runs a separate instance of Pedro 2.1.2's heading vector calculator
with the follower's heading PIDF coefficients, secondary-heading settings, and
radian units. It uses Pedro's derivative of error, not the custom measured-rate D.
It does not switch to hold-point mode, reset the follower, or change autonomous
constants. Its first active loop seeds derivative history and commands zero turn;
normal output begins on the next loop. History is reset on LB engagement,
controller changes, target-coordinate changes, and the +/-180-degree error wrap.
The extra minimum/kS settings also apply to Pedro mode. Its configured F is still
included, so use extra kS = 0 for an unmodified Pedro PIDF comparison.

Inside tolerance, Pedro output is zero once yaw rate is below the settle speed;
above that speed its PIDF output remains available to brake. The custom D setting
does not modify Pedro's gains. Test away from the Y = 72 target-switch boundary
first, then test both shooting targets.

Use telemetry for controller choice, custom P/D, target/error, yaw rate, base
controller output, final turn command, and minimum/kS. Compare both directions
from small and large errors with sticks centered, then while translating slowly.
Observe overshoot and time to settle, not just time to first cross the target.
Change one parameter at a time. Driver-triggered shooting remains unchanged.
