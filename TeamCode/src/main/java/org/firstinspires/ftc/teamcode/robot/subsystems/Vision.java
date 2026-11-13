package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import java.util.List;
import org.firstinspires.ftc.teamcode.robot.util.TipConfirmation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Vision extends SubsystemBase {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Robot.Alliance alliance;

    private final TipConfirmation tipConfirmation = new TipConfirmation();

    public boolean isAllianceVisible() {
        return containsAlliance(aprilTag.getDetections());
    }

    private boolean containsAlliance(List<AprilTagDetection> detections) {
        String prefix = alliance == Robot.Alliance.RED ? "RED" : "BLUE";
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata != null && cluster.metadata.name != null
                    && cluster.metadata.name.startsWith(prefix)) return true;
        }
        return false;
    }

    // Observe only after reaching the shooting position. Never reuse old frames as votes.
    public Command checkForTip() {
        return new CommandBase() {
            { addRequirements(Vision.this); }

            @Override public void initialize() {
                tipConfirmation.reset();
                aprilTag.getFreshDetections(); // Discard the frame from before this check.
            }

            @Override public void execute() {
                if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    tipConfirmation.reset();
                    return;
                }
                List<AprilTagDetection> fresh = aprilTag.getFreshDetections();
                if (fresh != null) {
                    tipConfirmation.observe(containsAlliance(fresh), System.nanoTime());
                }
            }

            @Override public boolean isFinished() { return false; }
        }.withTimeout(1000);
    }

    public boolean isTipConfirmed() {
        return tipConfirmation.isConfirmed(System.nanoTime());
    }

    public Vision(HardwareMap hardwareMap, Robot.Alliance robotAlliance) {
        alliance = robotAlliance;

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();
    }
}
