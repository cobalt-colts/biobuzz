package org.firstinspires.ftc.teamcode.robot.util;

public final class Constants {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int SNAPSCRIPT_PIPELINE_INDEX = 0;
    public static final int LIMELIGHT_POLL_RATE_HZ = 100;
    public static final long MAX_RESULT_AGE_MS = 250;

    // Physical camera geometry. Fill every NaN before enabling this mode.
    // Measure from the lens, with the robot level. Image must be upright,
    // camera facing straight ahead (no yaw/roll), and FOV must match pipeline zoom.
    public static final boolean USE_PHYSICAL_CAMERA_OFFSETS = true;
    // Positive right, negative left of the intake centerline (e.g. -5 = 5 inches left).
    public static final double CAMERA_RIGHT_OF_INTAKE_INCHES = -5.25;
    // Positive in front of the intake opening, negative behind it.
    public static final double CAMERA_FORWARD_OF_INTAKE_INCHES = -3.5;
    public static final double CAMERA_HEIGHT_INCHES = 13;
    public static final double CAMERA_DOWN_PITCH_DEGREES = 15;
    public static final double CAMERA_HORIZONTAL_FOV_DEGREES = 54.5;
    public static final double CAMERA_VERTICAL_FOV_DEGREES = 42;
    // Approximate height of the detected pollen centers, not their floor contact.
    public static final double POLLEN_CENTER_HEIGHT_INCHES = 2;
    // Estimated cluster center distance in front of the intake opening.
    public static final double CLUSTER_STOP_DISTANCE_INCHES = 2.0;
    public static final double CLUSTER_SLOW_DISTANCE_INCHES = 12.0;
    // Upper bound on vision latency reported in telemetry.
    public static final double MAX_LATENCY_COMPENSATION_S = 0.15;

    // Pollen path (X in teleop). Measure the intake offsets from the robot center
    // (Pedro's tracked point) to the intake opening before relying on the stop point.
    public static final double INTAKE_FORWARD_OF_ROBOT_CENTER_INCHES = 9.0;
    public static final double INTAKE_RIGHT_OF_ROBOT_CENTER_INCHES = 0.0;
    public static final double ROBOT_FORWARD_LENGTH_INCHES = 16.0;
    public static final double ROBOT_LATERAL_WIDTH_INCHES = 16.0;
    // Loop samples averaged into one field-frame cluster estimate before planning.
    public static final int TARGET_LOCK_SAMPLES = 5;
    // Shorter moves just hold the end pose instead of building a path.
    public static final double MIN_PATH_LENGTH_INCHES = 1.0;

    public static final int CLUSTER_BLOB_COUNT_INDEX = 1;
    public static final int NORMALIZED_CLUSTER_X_INDEX = 2;
    public static final int NORMALIZED_CLUSTER_Y_INDEX = 3;

    public static final int PYTHON_PROTOCOL_INDEX = 7;
    public static final double PYTHON_PROTOCOL_ID = 260924.0;

    private Constants() {
    }
}
