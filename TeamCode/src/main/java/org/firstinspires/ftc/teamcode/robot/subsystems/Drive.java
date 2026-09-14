package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public final class Drive extends SubsystemBase {
    public Command recalibratePinpoint(Follower follower) {
        return new InstantCommand(() -> {
            follower.stop();
            follower.localizer.reset();
            follower.setPose(Pose.zero());
        });
    }
    public void driveFieldCentric(
            double forward,
            double strafe,
            double turn,
            Follower follower
    ) {
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turn,
                follower.pose().heading());
        follower.manual(powers);
    }
}
