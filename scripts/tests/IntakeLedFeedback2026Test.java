import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem2026;
import org.firstinspires.ftc.teamcode.subsystems.LEDControlSubsystem2026.LedPreset;

/** Production subsystem checks with simulated hardware, against the compiled TeamCode classes. */
public final class IntakeLedFeedback2026Test {
    private static int checks;

    public static void main(String[] args) throws Exception {
        IntakeWithSensorsSubsystem2026 intake = IntakeWithSensorsSubsystem2026.INSTANCE;
        double[] motorCommands = new double[2];
        double[] measuredTicks = new double[2];
        for (int port = 0; port < 2; port++) {
            final int index = port;
            DcMotorEx motor = fake(DcMotorEx.class, (name, values) -> {
                if (name.equals("getVelocity")) return measuredTicks[index];
                if (name.equals("setVelocity") || name.equals("setPower")) motorCommands[index] = (double) values[0];
                return null;
            });
            put(intake, port == 0 ? "intakeMotor" : "transferMotor", motor);
        }
        for (String name : new String[]{"ballAtShooterBeam", "ballInMiddleHighBeam",
                "ballInMiddleLowBeam", "ballAtIntakeEntryBeam"}) {
            put(intake, name, fake(DigitalChannel.class, (method, values) ->
                    method.equals("getState") ? true : null));
        }
        double[] gatePosition = new double[1];
        put(intake, "shooterGate", fake(Servo.class, (name, values) -> {
            if (name.equals("setPosition")) gatePosition[0] = (double) values[0];
            return name.equals("getPosition") ? gatePosition[0] : null;
        }));
        intake.stopAll();
        intake.intakeForward();
        intake.periodic();
        check("collecting commands both motors forward", motorCommands[0] > 0 && motorCommands[1] > 0);
        intake.setJamReversing(true);
        intake.periodic();
        check("manual B reverses both motors", motorCommands[0] < 0 && motorCommands[1] < 0);
        check("reverse keeps gate closed", gatePosition[0] == IntakeWithSensorsSubsystem2026.GATE_CLOSED_POSITION);
        intake.setJamReversing(false);
        intake.periodic();
        check("releasing B immediately restores forward intake", motorCommands[0] > 0 && motorCommands[1] > 0);
        intake.setFeedingToShooter(true);
        intake.periodic();
        check("normal feed opens gate", gatePosition[0] == IntakeWithSensorsSubsystem2026.GATE_OPEN_POSITION);
        intake.stopIntake();
        intake.periodic();
        check("stop cancels motion and closes gate", motorCommands[0] == 0 && motorCommands[1] == 0
                && gatePosition[0] == IntakeWithSensorsSubsystem2026.GATE_CLOSED_POSITION);

        intake.intakeForward();
        intake.setBallCount(2);
        measuredTicks[0] = 4000 * 28.0 / 60;
        intake.periodic();
        check("two pollen stop only transfer", motorCommands[0] > 0 && motorCommands[1] == 0);
        intake.setBallCount(4);
        intake.periodic();
        check("four pollen stop both without reversing", motorCommands[0] == 0 && motorCommands[1] == 0);
        intake.stopAll();

        LEDControlSubsystem2026 leds = LEDControlSubsystem2026.INSTANCE;
        double[] positions = new double[2];
        ServoControllerEx controller = fake(ServoControllerEx.class, (name, values) -> {
            if (name.equals("setServoPosition")) positions[(int) values[0]] = (double) values[1];
            return null;
        });
        // The fake controller ignores servo-type configuration; no hardware is mapped here.
        put(leds, "led1", new ServoImplEx(controller, 0, null));
        put(leds, "led2", new ServoImplEx(controller, 1, null));
        put(leds, "pwmMinimumUs", 500.0);
        put(leds, "pwmMaximumUs", 2500.0);
        LEDControlSubsystem2026.AUTOMATIC_BALL_COUNT_DISPLAY = false;
        leds.displayBallCount(3);
        leds.setShootingFeedback(false, true);
        check("alignment begins with white", leds.getActivePreset() == LedPreset.WHITE);
        check("white uses solid-color PWM range", leds.getTargetPulseUs() == 1925);
        check("both LED channels synchronized", positions[0] == positions[1]);
        put(leds, "shootAlignedStartNanos", System.nanoTime() - 375_000_000L);
        leds.periodic();
        check("alignment flashes off", leds.getActivePreset() == LedPreset.OFF);
        leds.setShootingFeedback(true, true);
        check("range overrides aligned off phase with red", leds.getActivePreset() == LedPreset.RED);
        put(leds, "shootAlignedStartNanos", System.nanoTime());
        leds.periodic();
        check("range also overrides white phase", leds.getActivePreset() == LedPreset.RED);
        leds.signalStageChange();
        check("stage pulse cannot hide range warning", leds.getActivePreset() == LedPreset.RED);
        leds.clearStagePulse();
        leds.setShootingFeedback(false, false);
        check("clearing feedback restores current pollen count", leds.getActivePreset() == LedPreset.LIGHT_BLUE);
        System.out.println("Passed " + checks + " intake/LED integration checks.");
    }

    private interface Calls { Object call(String name, Object[] values); }

    @SuppressWarnings("unchecked")
    private static <T> T fake(Class<T> type, Calls calls) {
        return (T) Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, (proxy, method, values) -> {
            Object result = calls.call(method.getName(), values);
            if (result != null || !method.getReturnType().isPrimitive()) return result;
            if (method.getReturnType() == boolean.class) return false;
            if (method.getReturnType() == double.class) return 0.0;
            if (method.getReturnType() == int.class) return 0;
            return null;
        });
    }

    private static Field field(Object object, String name) throws Exception {
        Field result = object.getClass().getDeclaredField(name);
        result.setAccessible(true);
        return result;
    }
    private static void put(Object object, String name, Object value) throws Exception { field(object, name).set(object, value); }
    private static void check(String label, boolean condition) {
        if (!condition) throw new AssertionError(label);
        checks++;
    }
}
