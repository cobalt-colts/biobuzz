package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.util.Constants;

public final class PollenAcquisition extends SubsystemBase {
    private final Limelight3A limelight;

    private boolean active;
    private boolean hasTarget;
    private double entryX;
    private double entryY;

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

    public double getEntryX() {
        return entryX;
    }

    public double getEntryY() {
        return entryY;
    }

    public Command acquire(Follower follower) {
        return new CommandBase() {
            {
                addRequirements(PollenAcquisition.this);
            }

            @Override
            public void initialize() {
                active = true;
                hasTarget = false;
                entryX = Double.NaN;
                entryY = Double.NaN;
            }

            @Override
            public void execute() {
                LLResult result = limelight.getLatestResult();
                double[] output = result.getPythonOutput();

                hasTarget = limelight.isConnected()
                        && result.isValid()
                        && result.getStaleness() <= Constants.MAX_RESULT_AGE_MS
                        && output.length > Constants.NORMALIZED_ENTRY_Y_INDEX
                        && output[Constants.CLUSTER_BLOB_COUNT_INDEX] > 0;

                if (!hasTarget) {
                    entryX = Double.NaN;
                    entryY = Double.NaN;
                    follower.manual(0, 0, 0);
                    return;
                }

                entryX = output[Constants.NORMALIZED_ENTRY_X_INDEX];
                entryY = output[Constants.NORMALIZED_ENTRY_Y_INDEX];

                if (!isNormalized(entryX) || !isNormalized(entryY)
                        || entryY >= Constants.ENTRY_Y_STOP) {
                    follower.manual(0, 0, 0);
                    return;
                }

                double xError = entryX - Constants.ENTRY_X_SETPOINT;
                double strafe = Math.abs(xError) <= Constants.ENTRY_X_DEADBAND
                        ? 0
                        : clamp(
                                xError * Constants.STRAFE_KP,
                                -Constants.MAX_STRAFE_POWER,
                                Constants.MAX_STRAFE_POWER
                        );

                follower.manual(Constants.APPROACH_POWER, strafe, 0);
            }

            @Override
            public void end(boolean interrupted) {
                follower.stop();
                active = false;
                hasTarget = false;
                entryX = Double.NaN;
                entryY = Double.NaN;
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    private static boolean isNormalized(double value) {
        // A real blob center cannot be exactly zero. Rejecting zero also makes an old
        // eight-value SnapScript fail safe instead of being treated as a target.
        return Double.isFinite(value) && value > 0 && value <= 1;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
