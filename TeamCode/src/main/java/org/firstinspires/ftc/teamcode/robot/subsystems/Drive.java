package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class Drive extends SubsystemBase {
    PIDController headingControl = new PIDController(.8,0,0);

    double lastCommandedHeading;

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
            double turnx,
            double turny,
            Follower follower
    ) {
        if (Double.isNaN(lastCommandedHeading)) {
            lastCommandedHeading = follower.pose().heading();
        }

        double currentHeading = follower.pose().heading();
        double joystickHeading = Math.atan2(turnx, turny);

        if (Math.hypot(turnx, turny) <= 0.05) {
            joystickHeading = lastCommandedHeading;
        } else {
            lastCommandedHeading = joystickHeading;

        }

        double headingError = AngleUnit.normalizeRadians(
                joystickHeading - currentHeading
        );

        double pidSetpoint = currentHeading + headingError;

        double turnAmount = Math.clamp(headingControl.calculate(currentHeading, pidSetpoint), -1.0, 1.0);

        if (Math.abs(Math.toDegrees(headingError)) < 5) {
            turnAmount = 0;
        } else {
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turnAmount,
                follower.pose().heading());
        follower.manual(powers);
    }
}
