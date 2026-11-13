package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.util.GoalState;
import org.firstinspires.ftc.teamcode.robot.Robot;

public final class Drive extends SubsystemBase {
    // cell locations
    public static final double HIVE_CELL_CENTER_OFFSET = 11.79166667;
    public static final Pose NON_FLOWER_RED_CELL = new Pose(70.75- HIVE_CELL_CENTER_OFFSET, 70.75- HIVE_CELL_CENTER_OFFSET);
    public static final Pose NON_FLOWER_BLUE_CELL = new Pose(70.75+ HIVE_CELL_CENTER_OFFSET, 70.75+ HIVE_CELL_CENTER_OFFSET);
    public static final Pose FLOWER_RED_CELL = new Pose(70.75- HIVE_CELL_CENTER_OFFSET, 70.75+ HIVE_CELL_CENTER_OFFSET);
    public static final Pose FLOWER_BLUE_CELL = new Pose(70.75+ HIVE_CELL_CENTER_OFFSET, 70.75- HIVE_CELL_CENTER_OFFSET);



    PIDController headingControl = new PIDController(1,0,0.075);

    double lastCommandedHeading = Double.NaN;
    double fieldHeadingOffset = 0;

    public boolean goalsnap = false;

    public Drive(Robot.Alliance alliance) {
        fieldHeadingOffset = alliance == Robot.Alliance.BLUE ? Math.PI : 0;
        headingControl.setTolerance(Math.toRadians(2));
    }

    public Command resetHeading(Follower follower) {
        return new InstantCommand(() -> {
            fieldHeadingOffset = follower.pose().heading();
            lastCommandedHeading = Double.NaN;
        });
    }
    public void driveFieldCentric(
            double forward,
            double strafe,
            double turnx,
            double turny,
            Follower follower,
            GoalState.Goal activeGoal
    ) {
        if (Double.isNaN(lastCommandedHeading)) {
            lastCommandedHeading = follower.pose().heading();
        }

        if (goalsnap) {
            double snapheading;

            switch (activeGoal) {
                case FLOWER_RED:
                    snapheading = Math.atan2(FLOWER_RED_CELL.y() - follower.pose().y(), FLOWER_RED_CELL.x() - follower.pose().x());
                    break;
                case FLOWER_BLUE:
                    snapheading = Math.atan2(FLOWER_BLUE_CELL.y() - follower.pose().y(), FLOWER_BLUE_CELL.x() - follower.pose().x());
                    break;
                case NON_FLOWER_RED:
                    snapheading = Math.atan2(NON_FLOWER_RED_CELL.y() - follower.pose().y(), NON_FLOWER_RED_CELL.x() - follower.pose().x());
                    break;
                case NON_FLOWER_BLUE:
                    snapheading = Math.atan2(NON_FLOWER_BLUE_CELL.y() - follower.pose().y(), NON_FLOWER_BLUE_CELL.x() - follower.pose().x());
                    break;
                default:
                    snapheading = Math.atan2(NON_FLOWER_RED_CELL.y() - follower.pose().y(), NON_FLOWER_RED_CELL.x() - follower.pose().x());
            }

            lastCommandedHeading = snapheading;

            double headingError = AngleUnit.normalizeRadians(snapheading - follower.pose().heading());
            double pidSetpoint = headingError + follower.pose().heading();

            double turnPower = Math.clamp(headingControl.calculate(follower.pose().heading(), pidSetpoint), -1.0, 1.0);

            DrivePowers powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turnPower,
                    follower.pose().heading() - fieldHeadingOffset);
            follower.manual(powers);
        } else {
            double currentHeading = follower.pose().heading();
            double joystickHeading = Math.atan2(turnx, turny) + fieldHeadingOffset;

            if (Math.hypot(turnx, turny) <= 0.1) {
                joystickHeading = lastCommandedHeading;
            } else {
                lastCommandedHeading = joystickHeading;

            }

            double headingError = AngleUnit.normalizeRadians(
                    joystickHeading - currentHeading
            );

            double pidSetpoint = currentHeading + headingError;

            double turnAmount = Math.clamp(headingControl.calculate(currentHeading, pidSetpoint), -1.0, 1.0);

            DrivePowers powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turnAmount,
                    follower.pose().heading() - fieldHeadingOffset);
            follower.manual(powers);
        }
    }
}
