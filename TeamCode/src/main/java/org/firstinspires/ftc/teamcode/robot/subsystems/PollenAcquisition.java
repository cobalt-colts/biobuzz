package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;

import org.firstinspires.ftc.teamcode.robot.util.Constants;
import org.firstinspires.ftc.teamcode.robot.util.CameraGeometry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public final class PollenAcquisition extends SubsystemBase {
    private final Limelight3A limelight;

    private CameraGeometry.Target intakeTarget;
    private LLResult latestResult;
    private double[] pythonOutput;
    private String targetStatus = "Waiting for camera";
    private boolean connected;
    private boolean active;
    private boolean hasTarget;
    private double clusterX = Double.NaN;
    private double clusterY = Double.NaN;
    private double latencySeconds;
    private String pathStatus = "Idle";

    public PollenAcquisition(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.setPollRateHz(Constants.LIMELIGHT_POLL_RATE_HZ);
        Weaver.setConfig(new PathConfig()
                .robotWidth(Constants.ROBOT_FORWARD_LENGTH_INCHES)
                .robotHeight(Constants.ROBOT_LATERAL_WIDTH_INCHES));
    }

    public void start() {
        limelight.pipelineSwitch(Constants.SNAPSCRIPT_PIPELINE_INDEX);
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public boolean isActive() {
        return active;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getClusterX() {
        return clusterX;
    }

    public double getClusterY() {
        return clusterY;
    }

    /** Latest validated target position in inches relative to the intake. */
    public CameraGeometry.Target getIntakeTarget() {
        return hasTarget ? intakeTarget : null;
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
        connected = limelight.isConnected();
        pythonOutput = latestResult == null ? null : latestResult.getPythonOutput();
        hasTarget = false;
        intakeTarget = null;
        clusterX = Double.NaN;
        clusterY = Double.NaN;
        latencySeconds = 0;

        if (!connected) {
            targetStatus = "Camera disconnected";
        } else if (latestResult == null) {
            targetStatus = "No result received";
        } else if (latestResult.getPipelineIndex() != Constants.SNAPSCRIPT_PIPELINE_INDEX) {
            targetStatus = "Wrong pipeline";
        } else if (latestResult.getStaleness() > Constants.MAX_RESULT_AGE_MS) {
            targetStatus = "Stale result";
        } else if (pythonOutput == null || pythonOutput.length <= Constants.PYTHON_PROTOCOL_INDEX) {
            targetStatus = "Missing Python output";
        } else if (pythonOutput[Constants.PYTHON_PROTOCOL_INDEX] != Constants.PYTHON_PROTOCOL_ID) {
            targetStatus = "Upload current SnapScript to pipeline 0";
        } else if (!Double.isFinite(pythonOutput[Constants.CLUSTER_BLOB_COUNT_INDEX])
                || pythonOutput[Constants.CLUSTER_BLOB_COUNT_INDEX] < 0) {
            targetStatus = "Invalid blob count";
        } else if (pythonOutput[Constants.CLUSTER_BLOB_COUNT_INDEX] == 0) {
            targetStatus = "No cluster detected";
        } else if (!isNormalized(pythonOutput[Constants.NORMALIZED_CLUSTER_X_INDEX])
                || !isNormalized(pythonOutput[Constants.NORMALIZED_CLUSTER_Y_INDEX])) {
            targetStatus = "Invalid cluster coordinates";
        } else {
            // Custom Python payload owns target validity; SDK isValid describes
            // Limelight's standard contour target, not this payload's schema.
            clusterX = pythonOutput[Constants.NORMALIZED_CLUSTER_X_INDEX];
            clusterY = pythonOutput[Constants.NORMALIZED_CLUSTER_Y_INDEX];
            hasTarget = true;
            latencySeconds = clamp((latestResult.getCaptureLatency() + latestResult.getTargetingLatency()
                    + latestResult.getStaleness()) / 1000.0, 0, Constants.MAX_LATENCY_COMPENSATION_S);
            targetStatus = "Tracking cluster";
            if (Constants.USE_PHYSICAL_CAMERA_OFFSETS) {
                intakeTarget = CameraGeometry.project(clusterX, clusterY,
                        Constants.CAMERA_HEIGHT_INCHES, Constants.POLLEN_CENTER_HEIGHT_INCHES,
                        Constants.CAMERA_DOWN_PITCH_DEGREES,
                        Constants.CAMERA_HORIZONTAL_FOV_DEGREES, Constants.CAMERA_VERTICAL_FOV_DEGREES,
                        Constants.CAMERA_FORWARD_OF_INTAKE_INCHES, Constants.CAMERA_RIGHT_OF_INTAKE_INCHES);
                hasTarget = intakeTarget != null
                        && Double.isFinite(Constants.CLUSTER_STOP_DISTANCE_INCHES)
                        && Double.isFinite(Constants.CLUSTER_SLOW_DISTANCE_INCHES)
                        && Constants.CLUSTER_STOP_DISTANCE_INCHES >= 0
                        && Constants.CLUSTER_SLOW_DISTANCE_INCHES > Constants.CLUSTER_STOP_DISTANCE_INCHES;
                targetStatus = hasTarget ? "Tracking relative to intake"
                        : "Invalid camera geometry / target ray / distance settings";
            }
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pollen status", targetStatus);
        telemetry.addData("Pollen positioning", Constants.USE_PHYSICAL_CAMERA_OFFSETS
                ? "Physical camera offsets" : "Image coordinates (camera offsets disabled)");
        telemetry.addData("Camera lateral offset (in)", Constants.CAMERA_RIGHT_OF_INTAKE_INCHES);
        if (intakeTarget != null) {
            telemetry.addData("Cluster from intake (in)", "forward=%.1f right=%.1f",
                    intakeTarget.forward, intakeTarget.right);
        }
        telemetry.addData("Limelight connected", connected);
        if (latestResult != null) {
            telemetry.addData("Limelight pipeline", "%d (%s)",
                    latestResult.getPipelineIndex(), latestResult.getPipelineType());
            telemetry.addData("Limelight age ms", latestResult.getStaleness());
            telemetry.addData("Limelight contour valid", latestResult.isValid());
        }
        telemetry.addData("Limelight Python", pythonOutput == null ? "missing"
                : Arrays.toString(Arrays.copyOf(pythonOutput, Math.min(8, pythonOutput.length))));
        telemetry.addData("Pollen path", pathStatus);
        telemetry.addData("Limelight latency ms", "%.0f", latencySeconds * 1000);
        telemetry.addData("Pollen cluster", hasTarget
                ? String.format(java.util.Locale.US, "%.2f, %.2f", clusterX, clusterY)
                : "unavailable");
    }

    /**
     * Locks onto the cluster once, builds a single Weaver path to it, and lets Pedro
     * follow that path to the end. The path is never re-planned while X is held, so
     * vision noise and the cluster leaving the camera view cannot make it wander.
     */
    public Command acquire(Follower follower) {
        return new CommandBase() {
            private double sumX;
            private double sumY;
            private int samples;
            private boolean planned;

            {
                addRequirements(PollenAcquisition.this);
            }

            @Override
            public void initialize() {
                active = true;
                sumX = 0;
                sumY = 0;
                samples = 0;
                planned = false;
                pathStatus = "Locking target";
                follower.manual(0, 0, 0);
            }

            @Override
            public void execute() {
                if (planned) {
                    pathStatus = follower.isBusy() ? "Following path" : "Holding at cluster";
                    return;
                }
                if (!hasTarget || intakeTarget == null) {
                    return;
                }

                // Average the cluster in field coordinates so the robot coasting to a
                // stop during the lock does not skew the estimate.
                Pose pose = follower.pose();
                double forward = Constants.INTAKE_FORWARD_OF_ROBOT_CENTER_INCHES + intakeTarget.forward;
                // CameraGeometry uses forward/right; Pedro's local +Y is left.
                double left = -(Constants.INTAKE_RIGHT_OF_ROBOT_CENTER_INCHES + intakeTarget.right);
                double cos = Math.cos(pose.heading());
                double sin = Math.sin(pose.heading());
                sumX += pose.x() + forward * cos - left * sin;
                sumY += pose.y() + forward * sin + left * cos;
                if (++samples < Constants.TARGET_LOCK_SAMPLES) {
                    return;
                }

                double clusterFieldX = sumX / samples;
                double clusterFieldY = sumY / samples;
                double heading = Math.atan2(clusterFieldY - pose.y(), clusterFieldX - pose.x());
                // Park the robot center so the intake faces the cluster at the stop distance.
                double back = Constants.INTAKE_FORWARD_OF_ROBOT_CENTER_INCHES
                        + Constants.CLUSTER_STOP_DISTANCE_INCHES;
                double right = Constants.INTAKE_RIGHT_OF_ROBOT_CENTER_INCHES;
                Pose end = new Pose(
                        clusterFieldX - back * Math.cos(heading) - right * Math.sin(heading),
                        clusterFieldY - back * Math.sin(heading) + right * Math.cos(heading),
                        heading);

                if (Math.hypot(end.x() - pose.x(), end.y() - pose.y()) < Constants.MIN_PATH_LENGTH_INCHES) {
                    follower.hold(end);
                } else {
                    PathResult result = Weaver.builder()
                            .start(PedroPathingConverter.toPoint(pose))
                            .end(PedroPathingConverter.toPoint(end))
                            .build();
                    // Constant heading: Pedro turns to the final heading immediately and
                    // holds one setpoint the whole way, instead of chasing the tangent.
                    follower.follow(PedroPathingConverter.toPath(result).constant(heading));
                }
                planned = true;
                pathStatus = "Following path";
            }

            @Override
            public void end(boolean interrupted) {
                follower.stop();
                active = false;
                pathStatus = "Idle";
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    private static boolean isNormalized(double value) {
        return Double.isFinite(value) && value >= 0 && value <= 1;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
