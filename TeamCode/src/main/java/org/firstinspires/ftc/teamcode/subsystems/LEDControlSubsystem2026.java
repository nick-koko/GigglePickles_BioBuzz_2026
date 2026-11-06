package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
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
 * Range warnings (red/off blink) override the progress bar. Stage pulses also
 * temporarily override it. The live ball count is retained underneath.
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

    // Red/off blink: seconds for each half-cycle.
    public static double SHOOT_RANGE_WARNING_BLINK_SECONDS = 0.18;

    private Servo led1;
    private Servo led2;
    private LedPreset activePreset = LedPreset.RED;
    private LedPreset requestedPreset = LedPreset.RED;
    private boolean stagePulseActive;
    private long stagePulseStartNanos;
    private double activeStagePulseOnSeconds;
    private double activeStagePulseGapSeconds;
    private LedPreset activeStagePulsePreset;
    private boolean shootRangeWarningActive;
    private long shootRangeWarningStartNanos;
    private int displayedBallCount;
    private double targetPulseUs;
    private double targetServoPosition;

    private LEDControlSubsystem2026() {
    }

    @Override
    public void initialize() {
        stagePulseActive = false;
        shootRangeWarningActive = false;
        led1 = ActiveOpMode.hardwareMap().get(Servo.class, "rgb_color_PWM1");
        led2 = ActiveOpMode.hardwareMap().get(Servo.class, "rgb_color_PWM2");
        configurePwmRange(led1);
        configurePwmRange(led2);
        displayBallCount(IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount());
    }

    @Override
    public void periodic() {
        if (AUTOMATIC_BALL_COUNT_DISPLAY) {
            displayBallCount(IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount());
        } else if (stagePulseActive || shootRangeWarningActive) {
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
        if (active && !shootRangeWarningActive) {
            shootRangeWarningStartNanos = System.nanoTime();
        }
        shootRangeWarningActive = active;
        updateDisplay();
    }

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
        if (shootRangeWarningActive) {
            double interval = Double.isFinite(SHOOT_RANGE_WARNING_BLINK_SECONDS)
                    ? Math.max(0.02, SHOOT_RANGE_WARNING_BLINK_SECONDS) : 0.18;
            double elapsed = (System.nanoTime() - shootRangeWarningStartNanos) / 1e9;
            boolean redPhase = ((long) (elapsed / interval) % 2) == 0;
            preset = redPhase ? LedPreset.RED : LedPreset.OFF;
        }
        activePreset = preset;
        setPulseWidthMicroseconds(activePreset.centerPulseUs() + GLOBAL_PULSE_TRIM_US);
    }

    public void setPulseWidthMicroseconds(double pulseUs) {
        double minimum = Math.min(PWM_MIN_US, PWM_MAX_US);
        double maximum = Math.max(PWM_MIN_US, PWM_MAX_US);
        if (maximum <= minimum) {
            return;
        }
        targetPulseUs = Range.clip(pulseUs, minimum, maximum);
        targetServoPosition = Range.clip(
                (targetPulseUs - minimum) / (maximum - minimum), 0.0, 1.0);
        if (led1 != null) {
            led1.setPosition(targetServoPosition);
        }
        if (led2 != null) {
            led2.setPosition(targetServoPosition);
        }
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

    private static void configurePwmRange(Servo servo) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(
                    new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
        }
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
