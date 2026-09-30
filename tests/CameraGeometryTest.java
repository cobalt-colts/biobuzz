import org.firstinspires.ftc.teamcode.robot.util.CameraGeometry;

/** Standalone regression checks; compile alongside CameraGeometry.java and run with java. */
public final class CameraGeometryTest {
    private static void near(double actual, double expected) {
        if (Math.abs(actual - expected) > 1e-8) throw new AssertionError(actual + " != " + expected);
    }

    public static void main(String[] args) {
        CameraGeometry.Target center = CameraGeometry.project(.5, .5, 12, 2, 45, 90, 90, -3, -4);
        near(center.forward, 7);
        near(center.right, -4);
        CameraGeometry.Target right = CameraGeometry.project(.75, .5, 12, 2, 45, 90, 90, -3, -4);
        near(right.forward, 7);
        near(right.right, -4 + 5 * Math.sqrt(2));
        CameraGeometry.Target lower = CameraGeometry.project(.5, .75, 12, 2, 45, 90, 90, 0, 0);
        near(lower.forward, 10.0 / 3);
        CameraGeometry.Target centeredIntake = CameraGeometry.project(
                .5 + 4 / (20 * Math.sqrt(2)), .5, 12, 2, 45, 90, 90, 0, -4);
        near(centeredIntake.right, 0);
        if (CameraGeometry.project(.5, .5, 12, 2, 0, 90, 90, 0, 0) != null)
            throw new AssertionError("Horizon accepted");
        if (CameraGeometry.project(.5, 0, 12, 2, 10, 90, 90, 0, 0) != null)
            throw new AssertionError("Above horizon accepted");
        if (CameraGeometry.project(.5, .5, Double.NaN, 2, 45, 90, 90, 0, 0) != null)
            throw new AssertionError("Missing calibration accepted");
        if (CameraGeometry.project(.5, .5, 2, 2, 45, 90, 90, 0, 0) != null)
            throw new AssertionError("Invalid height accepted");
        if (CameraGeometry.project(.5, .5, 12, 2, 45, 180, 90, 0, 0) != null)
            throw new AssertionError("Invalid FOV accepted");
        System.out.println("PASS: mounting offsets, target height, image projection, intake alignment, invalid geometry");
    }
}
