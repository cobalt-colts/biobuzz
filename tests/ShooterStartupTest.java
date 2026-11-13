/**
 * Standalone startup regression check. Compile with javac, then run with the compiled
 * TeamCode classes plus SolversLib core and FTC RobotCore classes.jar on the classpath.
 * Does not construct hardware or start an OpMode.
 */
public final class ShooterStartupTest {
    public static void main(String[] args) throws Exception {
        // Dashboard scanning reads static fields before any OpMode is initialized.
        // Invalid static lookup tables must fail this check instead of crashing the hub.
        Class<?> shooter = Class.forName("org.firstinspires.ftc.teamcode.robot.subsystems.Shooter");
        for (String field : new String[]{"kp", "ki", "kd", "kf", "speed"}) {
            Object value = shooter.getField(field).get(null);
            if (!(value instanceof Number) || !Double.isFinite(((Number) value).doubleValue())) {
                throw new AssertionError("Invalid shooter setting: " + field);
            }
        }
        System.out.println("PASS: Shooter initializes and dashboard settings are readable");
    }
}
