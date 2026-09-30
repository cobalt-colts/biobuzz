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
    public static final double INTAKE_STRAFE_KP = 0.035;
    public static final double MAX_INTAKE_STRAFE_POWER = 0.20;

    // Tune this to the intake center in the camera image when the camera is offset.
    public static final double CLUSTER_X_SETPOINT = 0.5;
    public static final double CLUSTER_X_DEADBAND = 0.025;
    // Image position is a proximity cue, not a measured distance. Tune on the robot.
    public static final double CLUSTER_Y_SLOW = 0.80;
    public static final double CLUSTER_Y_STOP = 0.94;

    public static final double APPROACH_POWER = 0.60;
    public static final double MIN_APPROACH_POWER = 0.20;
    // Rightward intake alignment bias as a fraction of forward power.
    public static final double APPROACH_RIGHT_STRAFE_RATIO = 0.20;
    public static final double TURN_KP = 1.5;
    public static final double MAX_TURN_POWER = 0.50;
    // Positive Pedro turn is counterclockwise; positive image error is to the right.
    public static final double TURN_DIRECTION = -1.0;
    // Turn in place outside this horizontal error (fraction of image width).
    public static final double TURN_ONLY_ERROR = 0.35;

    public static final int CLUSTER_BLOB_COUNT_INDEX = 1;
    public static final int NORMALIZED_CLUSTER_X_INDEX = 2;
    public static final int NORMALIZED_CLUSTER_Y_INDEX = 3;

    public static final int PYTHON_PROTOCOL_INDEX = 7;
    public static final double PYTHON_PROTOCOL_ID = 260924.0;

    private Constants() {
    }
}
