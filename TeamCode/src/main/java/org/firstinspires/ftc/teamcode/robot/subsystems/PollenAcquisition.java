package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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

    public PollenAcquisition(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.setPollRateHz(Constants.LIMELIGHT_POLL_RATE_HZ);
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

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
        connected = limelight.isConnected();
        pythonOutput = latestResult == null ? null : latestResult.getPythonOutput();
        hasTarget = false;
        intakeTarget = null;
        clusterX = Double.NaN;
        clusterY = Double.NaN;

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
        telemetry.addData("Pollen cluster", hasTarget
                ? String.format(java.util.Locale.US, "%.2f, %.2f", clusterX, clusterY)
                : "unavailable");
    }

    public Command acquire(Follower follower) {
        return new CommandBase() {
            {
                addRequirements(PollenAcquisition.this);
            }

            @Override
            public void initialize() {
                active = true;
                follower.manual(0, 0, 0);
            }

            @Override
            public void execute() {
                if (!hasTarget) {
                    follower.manual(0, 0, 0);
                    return;
                }

                double xError = intakeTarget == null ? clusterX - Constants.CLUSTER_X_SETPOINT
                        : Math.atan2(intakeTarget.right, Math.max(0.01, intakeTarget.forward))
                                / Math.toRadians(Constants.CAMERA_HORIZONTAL_FOV_DEGREES);
                double turn = Math.abs(xError) <= Constants.CLUSTER_X_DEADBAND
                        ? 0
                        : clamp(
                                Constants.TURN_DIRECTION * xError * Constants.TURN_KP,
                                -Constants.MAX_TURN_POWER,
                                Constants.MAX_TURN_POWER
                        );

                // Reduce translation while turning toward the target.
                double alignmentScale = clamp(
                        (Constants.TURN_ONLY_ERROR - Math.abs(xError))
                                / (Constants.TURN_ONLY_ERROR - Constants.CLUSTER_X_DEADBAND),
                        0, 1);
                double proximityScale = intakeTarget == null ? clamp(
                        (Constants.CLUSTER_Y_STOP - clusterY)
                                / (Constants.CLUSTER_Y_STOP - Constants.CLUSTER_Y_SLOW), 0, 1)
                        : clamp((intakeTarget.forward - Constants.CLUSTER_STOP_DISTANCE_INCHES)
                                / (Constants.CLUSTER_SLOW_DISTANCE_INCHES - Constants.CLUSTER_STOP_DISTANCE_INCHES), 0, 1);
                boolean closeEnough = intakeTarget == null ? clusterY >= Constants.CLUSTER_Y_STOP
                        : intakeTarget.forward <= Constants.CLUSTER_STOP_DISTANCE_INCHES;
                // At the intake threshold, stop translation but finish aligning.
                double forward = closeEnough ? 0
                        : alignmentScale * (Constants.MIN_APPROACH_POWER
                                + (Constants.APPROACH_POWER - Constants.MIN_APPROACH_POWER) * proximityScale);

                // Robot-centric vision correction through Pedro's manual drive mode.
                // Pedro positive strafe is left. Physical mode uses measured lateral
                // error instead of adding the fixed right bias a second time.
                double strafe = intakeTarget == null ? -forward * Constants.APPROACH_RIGHT_STRAFE_RATIO
                        : (forward <= 0 ? 0 : -clamp(intakeTarget.right * Constants.INTAKE_STRAFE_KP,
                                -Constants.MAX_INTAKE_STRAFE_POWER, Constants.MAX_INTAKE_STRAFE_POWER)
                                * alignmentScale);
                follower.manual(forward, strafe, turn);
            }

            @Override
            public void end(boolean interrupted) {
                follower.stop();
                active = false;
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
