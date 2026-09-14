package org.firstinspires.ftc.teamcode.robot.util;

public final class Constants {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int SNAPSCRIPT_PIPELINE_INDEX = 0;
    public static final int LIMELIGHT_POLL_RATE_HZ = 100;
    public static final long MAX_RESULT_AGE_MS = 250;

    // Tune this to the intake center in the camera image when the camera is offset.
    public static final double ENTRY_X_SETPOINT = 0.5;
    public static final double ENTRY_X_DEADBAND = 0.04;
    public static final double ENTRY_Y_STOP = 0.82;

    public static final double APPROACH_POWER = 0.28;
    public static final double STRAFE_KP = 0.65;
    public static final double MAX_STRAFE_POWER = 0.30;

    public static final int CLUSTER_BLOB_COUNT_INDEX = 1;
    public static final int NORMALIZED_ENTRY_X_INDEX = 8;
    public static final int NORMALIZED_ENTRY_Y_INDEX = 9;

    private Constants() {
    }
}
