package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Controls the two-position flower intake servo.
 *
 * <p>Initialization only maps the hardware. An OpMode must explicitly call
 * {@link #moveUp()} or {@link #moveDown()} so enabling a TeleOp does not move
 * the mechanism before Play is pressed.</p>
 */
@Configurable
public class FlowerIntakeSubsystem2026 implements Subsystem {

    public static final FlowerIntakeSubsystem2026 INSTANCE =
            new FlowerIntakeSubsystem2026();

    public static double DOWN_POSITION = 0.33;
    public static double UP_POSITION = 0.75;

    public enum Position {
        UNKNOWN,
        UP,
        DOWN
    }

    private Servo flowerIntake;
    private Position position = Position.UNKNOWN;

    private FlowerIntakeSubsystem2026() {
    }

    @Override
    public void initialize() {
        flowerIntake = ActiveOpMode.hardwareMap().get(
                Servo.class,
                "flower_intake"
        );
        position = Position.UNKNOWN;
    }

    public void moveUp() {
        setPosition(UP_POSITION, Position.UP);
    }

    public void moveDown() {
        setPosition(DOWN_POSITION, Position.DOWN);
    }

    public void toggle() {
        if (position == Position.UP) {
            moveDown();
        } else {
            moveUp();
        }
    }

    public Position getPosition() {
        return position;
    }

    public double getServoPosition() {
        return flowerIntake == null
                ? Double.NaN
                : flowerIntake.getPosition();
    }

    private void setPosition(double servoPosition, Position newPosition) {
        if (flowerIntake == null) {
            return;
        }

        flowerIntake.setPosition(Range.clip(servoPosition, 0.0, 1.0));
        position = newPosition;
    }
}
