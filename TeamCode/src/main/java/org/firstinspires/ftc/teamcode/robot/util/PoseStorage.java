package org.firstinspires.ftc.teamcode.robot.util;

import android.os.SystemClock;
import com.pedropathing.math.Pose;

public final class PoseStorage {
    private static Pose savedPose = null;
    private static long savedAt = 0;
    private static final long MAX_AGE_MS = 90_000;

    private PoseStorage() {}

    public static void save(Pose pose) {
        savedPose = new Pose(pose.x(), pose.y(), pose.heading());
        savedAt = SystemClock.elapsedRealtime();
    }

    public static Pose consumeOrDefault(Pose defaultPose) {
        Pose result = defaultPose;

        if (savedPose != null &&
                SystemClock.elapsedRealtime() - savedAt < MAX_AGE_MS) {
            result = savedPose;
        }

        savedPose = null;
        savedAt = 0;

        return result;
    }
}