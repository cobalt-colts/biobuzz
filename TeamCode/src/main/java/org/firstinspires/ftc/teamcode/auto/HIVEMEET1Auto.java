package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.paths.HIVEMEET1Paths;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@Autonomous(name="HIVE MEET 1 Auto")
public class HIVEMEET1Auto extends CommandOpMode {
    Robot robot;
    Follower follower;

    HIVEMEET1Paths paths = new HIVEMEET1Paths();
    Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitCommand(4000),
                robot.transfer.runTransferForTime(2000),
                new ParallelCommandGroup(
                        robot.transfer.kick(),
                        robot.transfer.runTransferForTime(1000)
                ),
                robot.intake.setIntakeMotorPow(1),
                new FollowPathCommand(follower, paths.intakeline1(), .5),
                new FollowPathCommand(follower, paths.intakeline2()),
                new WaitCommand(1000),
                new FollowPathCommand(follower, paths.intakeback()),
                new FollowPathCommand(follower, paths.intakein(), 0.5),
                new WaitCommand(1000),
                new FollowPathCommand(follower, paths.shoot2()),
                robot.transfer.runTransferForTime(2000),
                new ParallelCommandGroup(
                        robot.transfer.kick(),
                        robot.transfer.runTransferForTime(1000)
                ),
                robot.shooter.killShooter(),
                new FollowPathCommand(follower, paths.park()),
                new FollowPathCommand(follower, paths.park2(), true),
                new InstantCommand(follower::stop)
        );
    }

    @Override
    public void initialize() {
        super.reset();
        robot = new Robot(hardwareMap, Robot.OpModeTypes.AUTO, telemetry, Robot.Alliance.RED);

        follower = Constants.create(hardwareMap);

        robot.intake.flowerStore().schedule();
    }

    @Override
    public void preRun() {
        Shooter.speed = 1270;
        follower.localizer.setPose(new Pose(58.6, 7.8, Math.toRadians(90)));
        robot.intake.flowerUp().schedule();
        autoRoutine().schedule();
    }

    @Override
    public void run() {
        super.run();

        follower.update();

        telemetry.update();
    }

    @Override
    public void end() {
        robot.intake.setIntakeMotorPow(0).initialize();
        robot.transfer.setTransferPower(0);
        follower.stop();
    }
}
