package org.firstinspires.ftc.teamcode.robot.util;

/** Pinhole estimate of a target on a horizontal plane; distances are inches. */
public final class CameraGeometry {
    private CameraGeometry() { }

    public static final class Target {
        public final double forward;
        public final double right;

        private Target(double forward, double right) {
            this.forward = forward;
            this.right = right;
        }
    }

    public static Target project(double x, double y, double height, double targetHeight,
                                 double pitchDegrees, double horizontalFov, double verticalFov,
                                 double cameraForward, double cameraRight) {
        double[] values = {x, y, height, targetHeight, pitchDegrees, horizontalFov,
                verticalFov, cameraForward, cameraRight};
        for (double value : values) {
            if (!Double.isFinite(value)) return null;
        }
        if (x < 0 || x > 1 || y < 0 || y > 1 || targetHeight < 0 || height <= targetHeight
                || pitchDegrees < 0 || pitchDegrees > 90
                || horizontalFov <= 0 || horizontalFov >= 180
                || verticalFov <= 0 || verticalFov >= 180) return null;

        double pitch = Math.toRadians(pitchDegrees);
        double imageRight = (2 * x - 1) * Math.tan(Math.toRadians(horizontalFov / 2));
        double imageDown = (2 * y - 1) * Math.tan(Math.toRadians(verticalFov / 2));
        double rayForward = Math.cos(pitch) - imageDown * Math.sin(pitch);
        double rayDown = Math.sin(pitch) + imageDown * Math.cos(pitch);
        // A ray at/above the horizon does not intersect the target plane ahead.
        if (rayDown <= 1e-6 || rayForward <= 0) return null;
        double scale = (height - targetHeight) / rayDown;
        double forward = cameraForward + scale * rayForward;
        double right = cameraRight + scale * imageRight;
        if (!Double.isFinite(forward) || !Double.isFinite(right)) return null;
        return new Target(forward, right);
    }
}
