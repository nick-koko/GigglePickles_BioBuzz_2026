import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026;

/** Production counting, capacity and teleop input checks with timed beam samples. */
public final class IntakeCounting2026Test {
    private static final IntakeWithSensorsSubsystem2026 intake=IntakeWithSensorsSubsystem2026.INSTANCE;
    private static final double[] commands=new double[2];
    private static int mask, reads, checks;
    private static double gate;
    private static Method apply;

    public static void main(String[] args) throws Exception {
        apply=intake.getClass().getDeclaredMethod("applyOutputs",long.class);apply.setAccessible(true);
        for (int port=0;port<2;port++) {
            final int index=port;
            DcMotorEx motor=fake(DcMotorEx.class,(name,values)->{
                if (name.equals("setPower") || name.equals("setVelocity")) { commands[index]=(double)values[0];return null; }
                if (name.equals("getPower")) return commands[index];
                throw new AssertionError("Unexpected motor call "+name);
            });
            put(port==0?"intakeMotor":"transferMotor",motor);
        }
        String[] names={"ballAtShooterBeam","ballInMiddleHighBeam","ballInMiddleLowBeam","ballAtIntakeEntryBeam"};
        for (int bit=0;bit<4;bit++) {
            final int b=bit;
            put(names[bit],fake(DigitalChannel.class,(name,values)->{
                if (name.equals("getState")) { reads++;return (mask & (1<<b))==0; }
                throw new AssertionError("Unexpected beam call "+name);
            }));
        }
        put("shooterGate",fake(Servo.class,(name,values)->{
            if (name.equals("setPosition")) {gate=(double)values[0];return null;}
            if (name.equals("getPosition")) return gate;
            throw new AssertionError("Unexpected gate call "+name);
        }));
        check("all motor speed settings retained",IntakeWithSensorsSubsystem2026.INTAKE_FORWARD_RPM==4000
                && IntakeWithSensorsSubsystem2026.TRANSFER_FORWARD_RPM==700
                && IntakeWithSensorsSubsystem2026.INTAKE_REVERSE_RPM==3000
                && IntakeWithSensorsSubsystem2026.TRANSFER_REVERSE_RPM==700);
        matchFourReplay(false);
        matchFourReplay(true);
        independentConfirmationAndHoles();
        firstBallOverlapCannotRecoverExtraBalls();
        noInventedRepeatedCounts();
        releaseAndStopPriority();
        feedReverseAndDiagnostics();
        recoveryCanBeDisabled();
        longerConfirmationIsRespected();
        invalidConfiguration();
        System.out.println("Passed "+checks+" beam counting/controller-2 intake checks, including the Match 4 transition replay.");
    }

    private static void matchFourReplay(boolean recovery) throws Exception {
        reset(1,1);intake.intakeForward();
        IntakeWithSensorsSubsystem2026.ENABLE_SUSTAINED_BEAM_RECOVERY=recovery;
        sample(29.815862,1);sample(30.2,1); // Original one-shot post-reset resync has finished.
        sample(32.260520,9);
        sample(32.329680,5);
        sample(32.497999,13);
        sample(32.569903,7);
        check("recorded middle-high edge reaches two",intake.getBallCount()==2);
        check("transfer stops at two",commands[1]==0 && commands[0]>0);
        sample(32.597886,15);
        sample(32.631,15);sample(32.665,15);
        check("full pattern is not accepted before confirmation",intake.getBallCount()==2);
        sample(32.698,15);
        check("same beams reproduce old undercount or recover full stack",intake.getBallCount()==(recovery?4:2));
        check("new recovery stops intake, legacy case stays on",(commands[0]==0)==recovery);
        check("transfer remains stopped",commands[1]==0);
        int recovered=intake.getBallCountRecoveryEvents();
        sample(33.554755,7);sample(33.590424,15);sample(34,15);sample(35.5,15);
        check("entry hole never lowers count",intake.getBallCount()==(recovery?4:2));
        check("held beams do not repeat recovery",intake.getBallCountRecoveryEvents()==recovered);
        if (recovery) {
            check("recovery diagnostics retain from/to/mask",intake.getLastRecoveryFrom()==2
                    && intake.getLastRecoveryTo()==4 && intake.getLastRecoveryBeamMask()==15);
        }
        intake.setFeedingToShooter(true);sample(35.552090,15);
        check("feeding clears tracked occupancy despite still-blocked beams",intake.getBallCount()==0);
        sample(35.591976,15);sample(35.719387,10);sample(36.268067,0);
        check("no counting during commanded feed",intake.getBallCount()==0);
        check("shoot feeding power unchanged",commands[0]==.5 && commands[1]==.5);
        intake.setFeedingToShooter(false);sample(36.780504,0);
        check("collection resumes after feed",commands[0]>0 && commands[1]>0);
    }

    private static void independentConfirmationAndHoles() throws Exception {
        reset(1,1);intake.intakeForward();sample(0,1);
        sample(.01,5);sample(.04,7);sample(.08,15);sample(.12,7);sample(.15,7);
        check("entry blips do not reset upper three confirmation",intake.getBallCount()==3);
        check("three balls stop transfer but allow intake",commands[1]==0 && commands[0]>0);
        sample(.20,15);sample(.24,7);sample(.26,15);sample(.37,15);
        check("fourth position waits for its own continuous blockage",intake.getBallCount()==4 && commands[0]==0);
        sample(.40,0);sample(.60,0);
        check("holes cannot remove latched occupancy",intake.getBallCount()==4);
    }

    private static void firstBallOverlapCannotRecoverExtraBalls() throws Exception {
        // Reload before clip volley 2: middle-high precedes shooter, and the
        // existing 100 ms recovery used to count two at this point.
        reset(0,0);intake.intakeForward();
        sample(11.964,2);sample(12.193,3);sample(12.285,3);sample(12.310,3);
        check("first ball overlapping upper beams remains one",intake.getBallCount()==1);
        check("overlap does not stop transfer",commands[1]>0);
        sample(12.330,1);sample(12.446,1);
        check("first ball reaching its position retains one",intake.getBallCount()==1);
        sample(13,3);
        check("new middle-high blockage can count second ball",intake.getBallCount()==2 && commands[1]==0);

        // Reloads before clip volleys 4/5: lower ball and upper ball can cover
        // the top three beams while the first is still advancing.
        reset(0,0);intake.intakeForward();
        sample(47.812,2);sample(48.037,6);sample(48.069,7);sample(48.171,7);
        check("overlapping arrival cannot recover from one to three",intake.getBallCount()==1 && commands[1]>0);
        sample(48.3,1);sample(48.5,1);sample(49,7);sample(49.11,7);
        check("recovery resumes for newly arriving stack",intake.getBallCount()==3);
        sample(49.2,15);sample(49.31,15);
        check("fourth ball still stops intake after valid upper arrivals",intake.getBallCount()==4 && commands[0]==0);
    }
    private static void noInventedRepeatedCounts() throws Exception {
        reset(0,0);intake.intakeForward();sample(0,0);sample(.01,8);sample(.5,8);
        check("isolated lower beam cannot invent top-down balls",intake.getBallCount()==0);
        sample(.6,1);sample(1,1);sample(10,1);
        check("one stuck upper beam cannot be counted repeatedly",intake.getBallCount()==1);
        sample(11,15);sample(11.11,15);int events=intake.getBallCountRecoveryEvents();
        sample(12,15);sample(20,15);
        check("a persistently blocked full stack is bounded at four",intake.getBallCount()==4 && intake.getBallCountRecoveryEvents()==events);
        int before=reads;
        intake.getCachedBeamMask();intake.getConfirmedBeamStackCount();intake.getBeamBlockedSeconds(3);
        check("logging diagnostics add no beam reads",reads==before);
    }

    private static void releaseAndStopPriority() throws Exception {
        reset(0,4);
        intake.updateTeleopCollectionInputs(true,true,false,false,false);sample(0,0);
        check("controller-2 press starts forward even when full",intake.isIntakeCapacityOverrideActive() && commands[0]>0);
        check("held A never bypasses transfer capacity",commands[1]==0);
        check("manual collection keeps gate closed",gate==IntakeWithSensorsSubsystem2026.GATE_CLOSED_POSITION);
        intake.updateTeleopCollectionInputs(false,false,false,false,false);sample(.1,0);
        check("release restores intake capacity stop",commands[0]==0 && !intake.isIntakeCapacityOverrideActive());
        check("release does not change selected forward mode",intake.getIntakeMode()==IntakeWithSensorsSubsystem2026.IntakeMode.FORWARD);
        intake.updateTeleopCollectionInputs(false,true,false,false,false);sample(.2,0);
        check("continued held request can force full intake only",commands[0]>0 && commands[1]==0);
        intake.updateTeleopCollectionInputs(true,true,true,true,true);sample(.3,0);
        check("X wins over A, B and flower input",commands[0]==0 && commands[1]==0
                && !intake.isIntakeCapacityOverrideHeld());
        intake.updateTeleopCollectionInputs(false,true,false,false,false);sample(.4,0);
        check("holding A after X does not restart stopped intake",commands[0]==0 && commands[1]==0);
        intake.updateTeleopCollectionInputs(false,false,false,false,false);sample(.5,0);
        check("releasing A after X cannot start intake",commands[0]==0);
        intake.updateTeleopCollectionInputs(true,true,false,false,false);sample(.6,0);
        check("fresh A press after X starts again",commands[0]>0 && commands[1]==0);
        reset(0,0);
        intake.updateTeleopCollectionInputs(true,true,false,false,false);sample(0,0);
        intake.updateTeleopCollectionInputs(false,false,false,false,false);sample(.1,0);
        check("tap stays in normal collection when below capacity",commands[0]>0 && commands[1]>0);
        reset(0,4);
        intake.updateTeleopCollectionInputs(true,false,false,false,false);sample(0,0);
        check("controller-1 start has no capacity bypass",commands[0]==0 && commands[1]==0);
        intake.stopAll();sample(.1,0);
        check("opmode stop clears override",!intake.isIntakeCapacityOverrideHeld() && commands[0]==0);
    }

    private static void feedReverseAndDiagnostics() throws Exception {
        reset(15,0);intake.intakeForward();
        intake.setFeedingToShooter(true);sample(0,15);sample(1,15);
        check("long feed cannot pre-confirm collection",intake.getBallCount()==0 && intake.getConfirmedBeamStackCount()==0);
        intake.setFeedingToShooter(false);sample(1.1,15);sample(1.15,15);
        check("post-feed stack needs fresh confirmation",intake.getBallCount()==0);
        sample(1.21,15);check("remaining stack recovers after feed",intake.getBallCount()==4);
        intake.updateTeleopCollectionInputs(false,true,false,true,false);sample(1.3,15);
        check("B reverse wins over held A",commands[0]<0 && commands[1]<0 && !intake.isIntakeCapacityOverrideActive());
        check("reverse closes gate",gate==IntakeWithSensorsSubsystem2026.GATE_CLOSED_POSITION);
        intake.updateTeleopCollectionInputs(false,true,false,false,false);sample(1.4,15);
        check("B release returns to intake-only override",commands[0]>0 && commands[1]==0);
        reset(15,0);intake.runOnlyIntakeForward();sample(0,15);sample(1,15);
        check("diagnostic motion does not count balls",intake.getBallCount()==0);
    }

    private static void recoveryCanBeDisabled() throws Exception {
        reset(1,1);intake.intakeForward();sample(0,1);sample(.3,1);
        IntakeWithSensorsSubsystem2026.ENABLE_SUSTAINED_BEAM_RECOVERY=false;
        sample(1,5);sample(1.1,7);sample(1.2,15);sample(2,15);
        check("disabled recovery retains edge-only undercount",intake.getBallCount()==2);
        IntakeWithSensorsSubsystem2026.ENABLE_SUSTAINED_BEAM_RECOVERY=true;
        sample(2.1,15);check("enabling recovery reconciles already-held beams",intake.getBallCount()==4);
    }

    private static void invalidConfiguration() {
        IntakeWithSensorsSubsystem2026.SUSTAINED_BEAM_CONFIRM_SECONDS=Double.NaN;
        boolean failed=false;
        try { IntakeWithSensorsSubsystem2026.validateConfiguration(); } catch (IllegalArgumentException e) { failed=true; }
        check("invalid confirmation setting fails clearly at validation",failed);
        IntakeWithSensorsSubsystem2026.SUSTAINED_BEAM_CONFIRM_SECONDS=.1;
    }

    private static void longerConfirmationIsRespected() throws Exception {
        reset(15,0);intake.intakeForward();
        IntakeWithSensorsSubsystem2026.SUSTAINED_BEAM_CONFIRM_SECONDS=.5;
        sample(0,15);sample(.3,15);
        check("legacy resync cannot bypass the new configured delay",intake.getBallCount()==0);
        sample(.51,15);check("longer confirmed stack eventually recovers",intake.getBallCount()==4);
        IntakeWithSensorsSubsystem2026.SUSTAINED_BEAM_CONFIRM_SECONDS=.1;
    }

    private static void reset(int beams,int count) {
        mask=beams;intake.stopAll();intake.setBallCount(count);
        IntakeWithSensorsSubsystem2026.ENABLE_SUSTAINED_BEAM_RECOVERY=true;
    }
    private static void sample(double t,int beams) throws Exception {
        mask=beams;apply.invoke(intake,1_000_000_000L+Math.round(t*1e9));
    }
    private interface Calls { Object call(String method,Object[] args); }
    @SuppressWarnings("unchecked")
    private static <T>T fake(Class<T> type,Calls calls) {
        return (T)Proxy.newProxyInstance(type.getClassLoader(),new Class<?>[]{type},
                (p,m,a)->calls.call(m.getName(),a));
    }
    private static void put(String name,Object value) throws Exception {
        Field f=intake.getClass().getDeclaredField(name);f.setAccessible(true);f.set(intake,value);
    }
    private static void check(String label,boolean condition) { checks++;if(!condition)throw new AssertionError(label); }
}
