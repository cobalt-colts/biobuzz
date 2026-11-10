package org.firstinspires.ftc.teamcode.robot.util;

import android.os.SystemClock;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;

public final class PoseStorage {
    private static Pose savedPose = null;
    private static Robot.Alliance savedAlliance = null;
    private static long savedAt = 0;
    private static final long MAX_AGE_MS = 90_000;
    public static final Pose DEFAULT_RED_TELEOP_POSE =
            new Pose(58.6, 7.8, Math.toRadians(90));
    public static final Pose DEFAULT_BLUE_TELEOP_POSE =
            new Pose(141.5 - DEFAULT_RED_TELEOP_POSE.x(), 141.5 - DEFAULT_RED_TELEOP_POSE.y(), DEFAULT_RED_TELEOP_POSE.heading() + Math.PI);

    private PoseStorage() {}

    public static void save(Pose pose, Robot.Alliance alliance) {
        savedPose = new Pose(pose.x(), pose.y(), pose.heading());
        savedAt = SystemClock.elapsedRealtime();
        savedAlliance = alliance;
    }

    public static Pose consumeOrDefault(Pose defaultPose, Robot.Alliance alliance) {
        Pose result = defaultPose;

        if (savedPose != null && savedAlliance == alliance &&
                SystemClock.elapsedRealtime() - savedAt < MAX_AGE_MS) {
            result = savedPose;
        }

        savedPose = null;
        savedAlliance = null;
        savedAt = 0;

        return result;
    }
}