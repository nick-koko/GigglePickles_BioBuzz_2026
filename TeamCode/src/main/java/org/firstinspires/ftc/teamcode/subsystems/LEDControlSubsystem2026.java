package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Controls both goBILDA Prism RGB drivers using the existing PWM connections.
 *
 * Uses factory PWM presets for reliable ball count display without
 * programming custom Artboards.
 *
 * Solid red range warnings override synchronized white aim flashes, stage
 * pulses, and ball count. The live ball count is retained underneath.
 */
@Configurable
public class LEDControlSubsystem2026 implements Subsystem {

    public static final LEDControlSubsystem2026 INSTANCE = new LEDControlSubsystem2026();

    public static double PWM_MIN_US = 500.0;
    public static double PWM_MAX_US = 2500.0;
    public static double GLOBAL_PULSE_TRIM_US = 0.0;

    public static boolean AUTOMATIC_BALL_COUNT_DISPLAY = true;
    // Factory PWM presets. These work without custom Artboard programming.
    public static LedPreset ZERO_BALL_PRESET = LedPreset.RED;
    public static LedPreset ONE_BALL_PRESET = LedPreset.ORANGE;
    public static LedPreset TWO_BALL_PRESET = LedPreset.YELLOW;
    public static LedPreset THREE_BALL_PRESET = LedPreset.LIGHT_BLUE;
    public static LedPreset FOUR_BALL_PRESET = LedPreset.SNAKE_8;

    public static boolean STAGE_PULSES_ENABLED = true;
    public static LedPreset STAGE_PULSE_PRESET = LedPreset.SOLID_GREEN;
    public static double STAGE_PULSE_ON_SECONDS = 0.15;
    public static double STAGE_PULSE_GAP_SECONDS = 0.10;

    // Two complete white/off flashes per second. Captured PWM range is set at Init.
    public static double SHOOT_ALIGNED_BLINK_SECONDS = 0.25;

    private ServoImplEx led1;
    private ServoImplEx led2;
    private double pwmMinimumUs, pwmMaximumUs;
    private LedPreset activePreset = LedPreset.RED;
    private LedPreset requestedPreset = LedPreset.RED;
    private boolean stagePulseActive;
    private long stagePulseStartNanos;
    private double activeStagePulseOnSeconds;
    private double activeStagePulseGapSeconds;
    private LedPreset activeStagePulsePreset;
    private boolean shootRangeWarningActive;
    private boolean shootAlignedActive;
    private long shootAlignedStartNanos;
    private int displayedBallCount;
    private double targetPulseUs;
    private double targetServoPosition;

    private LEDControlSubsystem2026() {
    }

    @Override
    public void initialize() {
        if (!Double.isFinite(PWM_MIN_US) || !Double.isFinite(PWM_MAX_US)
                || PWM_MIN_US <= 0 || PWM_MAX_US <= PWM_MIN_US
                || !Double.isFinite(GLOBAL_PULSE_TRIM_US)
                || !Double.isFinite(SHOOT_ALIGNED_BLINK_SECONDS)
                || SHOOT_ALIGNED_BLINK_SECONDS <= 0) {
            throw new IllegalArgumentException("LED PWM range, trim, or flash timing is invalid");
        }
        pwmMinimumUs = PWM_MIN_US;
        pwmMaximumUs = PWM_MAX_US;
        stagePulseActive = false;
        shootRangeWarningActive = false;
        shootAlignedActive = false;
        led1 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "rgb_color_PWM1");
        led2 = ActiveOpMode.hardwareMap().get(ServoImplEx.class, "rgb_color_PWM2");
        configurePwmRange(led1);
        configurePwmRange(led2);
        displayBallCount(IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount());
    }

    @Override
    public void periodic() {
        if (AUTOMATIC_BALL_COUNT_DISPLAY) {
            displayBallCount(IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount());
        } else if (stagePulseActive || shootRangeWarningActive || shootAlignedActive) {
            updateDisplay();
        }
    }

    public void displayBallCount(int ballCount) {
        displayedBallCount = Math.max(0, Math.min(4, ballCount));
        setPreset(presetForBallCount(displayedBallCount));
    }

    public void setPreset(LedPreset preset) {
        requestedPreset = preset == null ? LedPreset.RED : preset;
        updateDisplay();
    }

    /** Starts a non-blocking double pulse; a new stage restarts the indication. */
    public void signalStageChange() {
        if (!STAGE_PULSES_ENABLED) {
            return;
        }
        activeStagePulseOnSeconds = Math.max(0.0, STAGE_PULSE_ON_SECONDS);
        activeStagePulseGapSeconds = Math.max(0.0, STAGE_PULSE_GAP_SECONDS);
        activeStagePulsePreset = STAGE_PULSE_PRESET == null
                ? LedPreset.SOLID_GREEN : STAGE_PULSE_PRESET;
        stagePulseStartNanos = System.nanoTime();
        stagePulseActive = activeStagePulseOnSeconds > 0.0;
        if (AUTOMATIC_BALL_COUNT_DISPLAY) {
            periodic();
        } else {
            updateDisplay();
        }
    }

    public void clearStagePulse() {
        boolean wasActive = stagePulseActive;
        stagePulseActive = false;
        if (AUTOMATIC_BALL_COUNT_DISPLAY) {
            periodic();
        } else if (wasActive) {
            updateDisplay();
        }
    }

    public boolean isStagePulseActive() {
        return stagePulseActive;
    }

    /** Range warnings override pulses and ball count; underlying count still updates. */
    public void setShootRangeWarning(boolean active) {
        shootRangeWarningActive = active;
        updateDisplay();
    }

    /** Atomically update feedback so a range warning always wins over alignment. */
    public void setShootingFeedback(boolean rangeWarning, boolean aligned) {
        if (aligned && !shootAlignedActive) shootAlignedStartNanos = System.nanoTime();
        shootRangeWarningActive = rangeWarning;
        shootAlignedActive = aligned;
        updateDisplay();
    }

    public boolean isShootAlignedActive() { return shootAlignedActive; }

    public boolean isShootRangeWarningActive() {
        return shootRangeWarningActive;
    }

    private void updateDisplay() {
        LedPreset preset = requestedPreset;
        if (stagePulseActive) {
            double elapsedSeconds = (System.nanoTime() - stagePulseStartNanos) / 1e9;
            double secondPulseStart = activeStagePulseOnSeconds + activeStagePulseGapSeconds;
            double endSeconds = secondPulseStart + activeStagePulseOnSeconds;
            if (!STAGE_PULSES_ENABLED || elapsedSeconds >= endSeconds) {
                stagePulseActive = false;
            } else if (elapsedSeconds < activeStagePulseOnSeconds
                    || elapsedSeconds >= secondPulseStart) {
                preset = activeStagePulsePreset;
            }
        }
        if (shootAlignedActive) {
            if (!Double.isFinite(SHOOT_ALIGNED_BLINK_SECONDS)
                    || SHOOT_ALIGNED_BLINK_SECONDS <= 0) {
                throw new IllegalArgumentException("LED aligned flash half-cycle must be positive and finite");
            }
            double elapsed = (System.nanoTime() - shootAlignedStartNanos) / 1e9;
            boolean whitePhase = ((long) (elapsed / SHOOT_ALIGNED_BLINK_SECONDS) % 2) == 0;
            preset = whitePhase ? LedPreset.WHITE : LedPreset.OFF;
        }
        if (shootRangeWarningActive) preset = LedPreset.RED;
        activePreset = preset;
        setPulseWidthMicroseconds(activePreset.centerPulseUs() + GLOBAL_PULSE_TRIM_US);
    }

    public void setPulseWidthMicroseconds(double pulseUs) {
        if (!Double.isFinite(pulseUs)) throw new IllegalArgumentException("LED pulse must be finite");
        targetPulseUs = Range.clip(pulseUs, pwmMinimumUs, pwmMaximumUs);
        targetServoPosition = Range.clip(
                (targetPulseUs - pwmMinimumUs) / (pwmMaximumUs - pwmMinimumUs), 0.0, 1.0);
        led1.setPosition(targetServoPosition);
        led2.setPosition(targetServoPosition);
    }

    public LedPreset getActivePreset() {
        return activePreset;
    }

    public int getDisplayedBallCount() {
        return displayedBallCount;
    }

    public double getTargetPulseUs() {
        return targetPulseUs;
    }

    public double getTargetServoPosition() {
        return targetServoPosition;
    }

    private static LedPreset presetForBallCount(int ballCount) {
        switch (ballCount) {
            case 1:
                return ONE_BALL_PRESET;
            case 2:
                return TWO_BALL_PRESET;
            case 3:
                return THREE_BALL_PRESET;
            case 4:
                return FOUR_BALL_PRESET;
            case 0:
            default:
                return ZERO_BALL_PRESET;
        }
    }

    private void configurePwmRange(ServoImplEx servo) {
        servo.setPwmRange(new PwmControl.PwmRange(pwmMinimumUs, pwmMaximumUs));
    }

    /** Pulse centers in microseconds, goBILDA Prism PWM control table. */
    public enum LedPreset {
        CUSTOM_0(505.0),
        CUSTOM_1(515.0),
        CUSTOM_2(525.0),
        CUSTOM_3(535.0),
        CUSTOM_4(545.0),
        CUSTOM_5(555.0),
        CUSTOM_6(565.0),
        CUSTOM_7(575.0),
        OFF(1075.0),
        RED(1105.0),
        ORANGE(1200.0),
        YELLOW(1300.0),
        LIGHT_BLUE(1600.0),
        SOLID_GREEN(1500.0),
        // Prism solid-color range ends at 1949 us; above 1900 us selects white.
        WHITE(1925.0),
        SNAKE_1(2205.0),
        SNAKE_2(2215.0),
        SNAKE_3(2225.0),
        SNAKE_4(2235.0),
        SNAKE_5(2245.0),
        SNAKE_6(2255.0),
        SNAKE_7(2265.0),
        SNAKE_8(2275.0),
        SNAKE_9(2285.0),
        SNAKE_10(2295.0),
        SNAKE_11(2305.0),
        SNAKE_12(2315.0),
        SNAKE_13(2325.0),
        SNAKE_14(2335.0),
        SNAKE_15(2345.0);

        private final double centerPulseUs;

        LedPreset(double centerPulseUs) {
            this.centerPulseUs = centerPulseUs;
        }

        public double centerPulseUs() {
            return centerPulseUs;
        }
    }
}
