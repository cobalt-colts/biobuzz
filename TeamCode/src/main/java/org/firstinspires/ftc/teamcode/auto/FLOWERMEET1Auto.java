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
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.auto.paths.FLOWERMEET1Paths;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.util.PoseStorage;

@Autonomous(name="FLOWER MEET 1 Auto")
public class FLOWERMEET1Auto extends CommandOpMode {
    Robot robot;
    Follower follower;

    FLOWERMEET1Paths paths = new FLOWERMEET1Paths();
    Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitCommand(12000),
                robot.transfer.runTransferForTime(2000),
                new ParallelCommandGroup(
                        robot.transfer.kick(),
                        robot.transfer.runTransferForTime(1000)
                ),
                new FollowPathCommand(follower, paths.shootNpark(), true),
                new TurnToCommand(follower, Math.toRadians(0)),
                new FollowPathCommand(follower, paths.movethefuckoutoftheway()),
                new InstantCommand(follower::stop)
        );
    }

    @Override
    public void initialize() {
        super.reset();
        robot = new Robot(hardwareMap, Robot.OpModeTypes.AUTO, telemetry);

        follower = Constants.create(hardwareMap);

        robot.intake.flowerStore().schedule();
    }

    @Override
    public void preRun() {
        Shooter.speed = 1270;
        follower.localizer.setPose(new Pose(58.5, 133.2, Math.toRadians(-90)));
        robot.intake.flowerUp().schedule();
        autoRoutine().schedule();
    }

    @Override
    public void run() {
        // Poll button bindings and run subsystem periodic methods each loop.
        super.run();

        follower.update();

        telemetry.update();
    }

    @Override
    public void end() {
        PoseStorage.save(follower.pose());

        robot.intake.setIntakeMotorPow(0).initialize();
        robot.transfer.setTransferPower(0);
        follower.stop();
    }
}
