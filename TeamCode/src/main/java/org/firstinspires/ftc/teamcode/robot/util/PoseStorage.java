package org.firstinspires.ftc.teamcode.robot.util;

import android.os.SystemClock;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Auto -> TeleOp pose handoff. Auto and TeleOp run on different OpMode threads,
 * so the snapshot is an immutable object published through a volatile reference.
 */
public final class PoseStorage {
    private static final class Snapshot {
        final Pose pose;
        final Robot.Alliance alliance;
        final long savedAt;
        Snapshot(Pose pose, Robot.Alliance alliance, long savedAt) {
            this.pose = pose;
            this.alliance = alliance;
            this.savedAt = savedAt;
        }
    }

    private static volatile Snapshot snapshot = null;

    // Long enough to cover the auto->teleop transition plus field delays,
    // short enough that a pose from an earlier match/practice run isn't reused.
    private static final long MAX_AGE_MS = 5 * 60_000;

    public static final Pose DEFAULT_RED_TELEOP_POSE =
            new Pose(58.6, 7.8, Math.toRadians(90));
    public static final Pose DEFAULT_BLUE_TELEOP_POSE =
            new Pose(141.5 - DEFAULT_RED_TELEOP_POSE.x(), 141.5 - DEFAULT_RED_TELEOP_POSE.y(), DEFAULT_RED_TELEOP_POSE.heading() + Math.PI);

    private PoseStorage() {}

    /** Cheap enough to call every loop; keeps the latest pose even if auto is aborted. */
    public static void save(Pose pose, Robot.Alliance alliance) {
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) return;
        snapshot = new Snapshot(new Pose(pose.x(), pose.y(), pose.heading()), alliance, SystemClock.elapsedRealtime());
    }

    public static synchronized Pose consumeOrDefault(Pose defaultPose, Robot.Alliance alliance) {
        Snapshot s = snapshot;
        snapshot = null;
        if (s != null && s.alliance == alliance && SystemClock.elapsedRealtime() - s.savedAt < MAX_AGE_MS) {
            return s.pose;
        }
        return defaultPose;
    }

    public static boolean hasPose(Robot.Alliance alliance) {
        Snapshot s = snapshot;
        return s != null && s.alliance == alliance;
    }
}
