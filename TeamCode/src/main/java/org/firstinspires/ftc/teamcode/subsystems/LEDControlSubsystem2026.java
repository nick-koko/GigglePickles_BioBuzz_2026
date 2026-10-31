package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Controls both 2026 goBILDA Prism RGB drivers from the tracked ball count.
 *
 * <p>The LED subsystem reads intake state; the intake subsystem intentionally
 * has no LED dependency. Register both subsystems in an OpMode to enable the
 * automatic display.</p>
 */
@Configurable
public class LEDControlSubsystem2026 implements Subsystem {

    public static final LEDControlSubsystem2026 INSTANCE =
            new LEDControlSubsystem2026();

    public static double PWM_MIN_US = 500.0;
    public static double PWM_MAX_US = 2500.0;
    public static double GLOBAL_PULSE_TRIM_US = 0.0;

    public static boolean AUTOMATIC_BALL_COUNT_DISPLAY = true;
    public static LedPreset ZERO_BALL_PRESET = LedPreset.RED;
    public static LedPreset ONE_BALL_PRESET = LedPreset.ORANGE;
    public static LedPreset TWO_BALL_PRESET = LedPreset.YELLOW;
    public static LedPreset THREE_BALL_PRESET = LedPreset.LIGHT_BLUE;
    public static LedPreset FOUR_BALL_PRESET = LedPreset.SNAKE_8;

    public static boolean STAGE_PULSES_ENABLED = true;
    public static LedPreset STAGE_PULSE_PRESET = LedPreset.SOLID_GREEN;
    public static double STAGE_PULSE_ON_SECONDS = 0.15;
    public static double STAGE_PULSE_GAP_SECONDS = 0.10;

    private Servo led1;
    private Servo led2;
    private LedPreset activePreset = LedPreset.RED;
    private LedPreset requestedPreset = LedPreset.RED;
    private boolean stagePulseActive;
    private long stagePulseStartNanos;
    private double activeStagePulseOnSeconds;
    private double activeStagePulseGapSeconds;
    private LedPreset activeStagePulsePreset;
    private int displayedBallCount;
    private double targetPulseUs;
    private double targetServoPosition;

    private LEDControlSubsystem2026() {
    }

    @Override
    public void initialize() {
        stagePulseActive = false;
        led1 = ActiveOpMode.hardwareMap().get(Servo.class, "rgb_color_PWM1");
        led2 = ActiveOpMode.hardwareMap().get(Servo.class, "rgb_color_PWM2");
        configurePwmRange(led1);
        configurePwmRange(led2);
        displayBallCount(
                IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount()
        );
    }

    @Override
    public void periodic() {
        if (AUTOMATIC_BALL_COUNT_DISPLAY) {
            displayBallCount(
                    IntakeWithSensorsSubsystem2026.INSTANCE.getBallCount()
            );
        } else if (stagePulseActive) {
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
        // Ball-count updates continue beneath the pulse, including in the gap.
        activePreset = preset;
        setPulseWidthMicroseconds(
                activePreset.centerPulseUs() + GLOBAL_PULSE_TRIM_US
        );
    }

    public void setPulseWidthMicroseconds(double pulseUs) {
        double minimum = Math.min(PWM_MIN_US, PWM_MAX_US);
        double maximum = Math.max(PWM_MIN_US, PWM_MAX_US);
        targetPulseUs = Range.clip(pulseUs, minimum, maximum);
        targetServoPosition = Range.clip(
                (targetPulseUs - minimum) / (maximum - minimum),
                0.0,
                1.0
        );

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
                    new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US)
            );
        }
    }

    /**
     * Pulse centers from goBILDA Product Insight 3203-3118-0001.
     * Snake presets use the center of each documented 10-microsecond range.
     */
    public enum LedPreset {
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
