import java.io.BufferedReader;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.nio.file.Files;
import java.nio.file.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/** Offline replay of recorded sensor samples through compiled production intake classes. */
public final class IntakeLogReplay2026 {
    private static final class Robot {
        final Class<?> type;
        final Object instance;
        final Method apply;
        int mask;
        final double[] commands = new double[2];
        double gate;
        int beforeCommandsCount;
        Robot(String name) throws Exception {
            type=Class.forName(name);instance=type.getField("INSTANCE").get(null);
            apply=type.getDeclaredMethod("applyOutputs",long.class);apply.setAccessible(true);
            for(int i=0;i<2;i++) {
                final int port=i;
                put(i==0?"intakeMotor":"transferMotor",fake(DcMotorEx.class,(method,args)->{
                    if(method.equals("setPower")||method.equals("setVelocity")){commands[port]=(double)args[0];return null;}
                    if(method.equals("getPower"))return commands[port];
                    if(method.equals("getVelocity"))return 0.0;
                    throw new AssertionError(method);
                }));
            }
            String[] names={"ballAtShooterBeam","ballInMiddleHighBeam","ballInMiddleLowBeam","ballAtIntakeEntryBeam"};
            for(int bit=0;bit<4;bit++) {
                final int b=bit;
                put(names[bit],fake(DigitalChannel.class,(method,args)->{
                    if(method.equals("getState"))return (mask&(1<<b))==0;
                    throw new AssertionError(method);
                }));
            }
            put("shooterGate",fake(Servo.class,(method,args)->{
                if(method.equals("setPosition")){gate=(double)args[0];return null;}
                if(method.equals("getPosition"))return gate;
                throw new AssertionError(method);
            }));
            call("stopAll");
            call("intakeForward");
        }
        void put(String name,Object value) throws Exception {
            Field field=type.getDeclaredField(name);field.setAccessible(true);field.set(instance,value);
        }
        Object call(String name) throws Exception { return type.getMethod(name).invoke(instance); }
        Object call(String name,Class<?> argType,Object arg) throws Exception {
            return type.getMethod(name,argType).invoke(instance,arg);
        }
        void advance(String[] row) throws Exception {
            mask=Integer.parseInt(row[2]);
            // Match 4 auto's mode transitions precede its subsystem sample;
            // feed commands follow the sample. Teleop's collection mode stays
            // FORWARD throughout. Historical motor outputs validate this ordering.
            String mode=row[5];
            if(!call("getIntakeMode").toString().equals(mode)) {
                if(mode.equals("FORWARD"))call("intakeForward");else call("stopIntake");
            }
            apply.invoke(instance,Long.parseLong(row[1]));
            beforeCommandsCount=(Integer)call("getBallCount");
            // Rising feed clears occupancy immediately; feed motor outputs
            // change on the following controller sample, as in the recorded data.
            call("setJamReversing",boolean.class,Boolean.parseBoolean(row[4]));
            call("setFeedingToShooter",boolean.class,Boolean.parseBoolean(row[3]));
        }
        String columns(boolean current) throws Exception {
            String values=call("getBallCount")+","+call("getAppliedIntakeRpm")+","+call("getAppliedTransferRpm")
                    +","+call("getAppliedControlMode")+","+call("isIntakeStoppedForCapacity")
                    +","+call("isTransferStoppedForCapacity");
            return current?values+","+call("getBallCountRecoveryEvents")+","+call("getConfirmedBeamStackCount")
                    +","+call("getLastRecoveryFrom")+","+call("getLastRecoveryTo"):values;
        }
    }

    public static void main(String[] args) throws Exception {
        Robot old=new Robot("org.firstinspires.ftc.teamcode.subsystems.ReplayLegacyIntake2026");
        Robot next=new Robot("org.firstinspires.ftc.teamcode.subsystems.IntakeWithSensorsSubsystem2026");
        if(args.length>2) next.type.getField("SUSTAINED_BEAM_CONFIRM_SECONDS").setDouble(null,Double.parseDouble(args[2]));
        try(BufferedReader input=Files.newBufferedReader(Path.of(args[0]));PrintWriter out=new PrintWriter(args[1])) {
            out.println("t_s,recorded_count,legacy_count,legacy_intake_rpm,legacy_transfer_rpm,legacy_control_mode,legacy_intake_stop,legacy_transfer_stop,new_count,new_intake_rpm,new_transfer_rpm,new_control_mode,new_intake_stop,new_transfer_stop,recovery_events,confirmed_stack,recovery_from,recovery_to,legacy_before_commands_count,new_before_commands_count");
            String line;int n=0,mismatch=0;
            while((line=input.readLine())!=null) {
                if(line.startsWith("t_s"))continue;
                String[] row=line.split("\\t");
                if(!row[6].equals("true")||!row[7].equals("NONE"))throw new AssertionError("Unsupported replay mode");
                if(n==0) for(Robot robot:new Robot[]{old,next}) {
                    robot.mask=Integer.parseInt(row[2]);
                    robot.call("setBallCount",int.class,Integer.parseInt(row[8]));
                    if(row[5].equals("STOPPED"))robot.call("stopIntake");
                }
                old.advance(row);next.advance(row);
                if(((Integer)old.call("getBallCount"))!=Integer.parseInt(row[8]))mismatch++;
                out.println(row[0]+","+row[8]+","+old.columns(false)+","+next.columns(true)
                        +","+old.beforeCommandsCount+","+next.beforeCommandsCount);n++;
            }
            System.out.println("Replayed "+n+" recorded loops; historical-count mismatches: "+mismatch);
            if(mismatch!=0)throw new AssertionError("Historical baseline did not reproduce: "+mismatch+" loops");
        }
    }
    private interface Calls {Object call(String method,Object[] args);}
    @SuppressWarnings("unchecked") private static <T>T fake(Class<T> type,Calls calls) {
        return (T)Proxy.newProxyInstance(type.getClassLoader(),new Class<?>[]{type},(p,m,a)->calls.call(m.getName(),a));
    }
}
