package org.firstinspires.ftc.teamcode.testing.shots;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.teamcode.logging.RobotLogging2026;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** Owns only configuration values, never hardware. Restore on completion, Stop and exceptions. */
public final class ScopedShotSettings2026 implements AutoCloseable {
    private final Map<Field,Object> original = new LinkedHashMap<>();
    private boolean closed;

    public ScopedShotSettings2026() {
        for (Class<?> type : new Class<?>[]{ShooterSubsystem2026.class,ShotSequenceController2026.class,
                IntakeWithSensorsSubsystem2026.class,LEDControlSubsystem2026.class,RobotLogging2026.class}) {
            for (Field field : type.getFields()) {
                if (!Modifier.isStatic(field.getModifiers()) || Modifier.isFinal(field.getModifiers())) continue;
                if (!(field.getType().isPrimitive() || field.getType().isEnum() || field.getType()==String.class)) continue;
                original.put(field,read(field));
            }
        }
    }

    public Object original(Class<?> owner,String name) { return original.get(field(owner,name)); }
    public double originalDouble(Class<?> owner,String name) { return ((Number)original(owner,name)).doubleValue(); }
    public void resetToBaseline() {
        if(closed)throw new IllegalStateException("Shot test settings scope is closed");
        restoreValues();
    }

    public void apply(ShotExperimentPlan2026.Profile profile) {
        if (closed) throw new IllegalStateException("Shot test settings scope is closed");
        restoreValues();
        for (Map.Entry<Field,Object> entry : profile.overrides.entrySet()) write(entry.getKey(),entry.getValue());
        ShooterSubsystem2026.validateConfiguration();
        ShotSequenceController2026.validateConfiguration();
        IntakeWithSensorsSubsystem2026.validateConfiguration();
        if (!Double.isFinite(IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER)
                || !Double.isFinite(IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER)
                || IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER<0 || IntakeWithSensorsSubsystem2026.SHOOT_INTAKE_POWER>1
                || IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER<0 || IntakeWithSensorsSubsystem2026.SHOOT_TRANSFER_POWER>1
                || ShotSequenceController2026.FEED_SECONDS<=0)
            throw new IllegalArgumentException("Shot test feed powers must be in [0,1] and feed duration positive");
    }

    private void restoreValues() {
        for (Map.Entry<Field,Object> entry : original.entrySet()) write(entry.getKey(),entry.getValue());
    }
    @Override public void close() { if (!closed) { restoreValues();closed=true; } }
    static Field field(Class<?> owner,String name) {
        try { return owner.getField(name); }
        catch (NoSuchFieldException e) { throw new IllegalArgumentException("Unknown shot test setting: "+owner.getSimpleName()+"."+name,e); }
    }
    private static Object read(Field field) {
        try { return field.get(null); }
        catch (IllegalAccessException e) { throw new IllegalStateException("Cannot snapshot "+field,e); }
    }
    private static void write(Field field,Object value) {
        try { field.set(null,value); }
        catch (IllegalAccessException e) { throw new IllegalStateException("Cannot restore/apply "+field,e); }
    }
}
