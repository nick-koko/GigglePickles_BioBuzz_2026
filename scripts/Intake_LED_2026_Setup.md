# Intake controls and driver LEDs

Automatic jam recovery has been removed. Hold B to reverse both intake motors
manually; release B to return to the selected intake mode. X stops the intake.

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
red/white visibility across the field and verify the white cue against the
physical shooter direction. Software checks use simulated hardware; the LED
alignment settings still need validation on the robot.
