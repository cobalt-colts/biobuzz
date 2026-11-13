package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.auto.paths.HIVEMEET1Paths;
import org.firstinspires.ftc.teamcode.auto.paths.HIVEMEET1TIPPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Tuning;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.util.PoseStorage;

@Autonomous(name="HIVE MEET 1 Auto")
public class HIVEMEET1Auto extends CommandOpMode {
    Robot robot;
    Follower follower;

    HIVEMEET1Paths paths = new HIVEMEET1Paths();
    HIVEMEET1TIPPaths tipPaths = new HIVEMEET1TIPPaths();

    Command tipRoutine() {
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, tipPaths.shoot()),
                new TurnToCommand(follower, Math.toRadians(-90)),
                new FollowPathCommand(follower, tipPaths.shoot2()),
                robot.intake.setIntakeMotorPow(1),
                robot.transfer.runTransferForTime(1250),
                new ParallelCommandGroup(
                        robot.transfer.kick(),
                        robot.transfer.runTransferForTime(500)
                ),
                robot.intake.setIntakeMotorPow(0),
                new FollowPathCommand(follower, tipPaths.flower1()),
                robot.intake.setIntakeMotorPow(1),
                robot.intake.flowerDown(),
                new FollowPathCommand(follower, tipPaths.flower2(), .25),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                robot.intake.flowerUp()
                        ),
                        new FollowPathCommand(follower, tipPaths.shoot3())
                ),
                robot.transfer.runTransferForTime(1250),
                new ParallelCommandGroup(
                        robot.transfer.kick(),
                        robot.transfer.runTransferForTime(500)
                ),
                new FollowPathCommand(follower, tipPaths.park(), true),
                new TurnToCommand(follower, 0)
        );
    }

    Command noTipRoutine() {
        return new SequentialCommandGroup(
                robot.intake.setIntakeMotorPow(1),
                robot.transfer.runTransferForTime(1500),
                new ParallelCommandGroup(
                        robot.transfer.kick(),
                        robot.transfer.runTransferForTime(500)
                ),
                robot.shooter.killShooter(),
                new FollowPathCommand(follower, paths.flower()),
                robot.intake.setIntakeMotorPow(1),
                robot.intake.flowerDown(),
                new FollowPathCommand(follower, paths.flower2(), .25),
                new WaitCommand(2000),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.park(), true),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                robot.intake.flowerStore()
                        )
                )
        );
    }
    Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitCommand(2000),
                robot.transfer.runTransferForTime(1250),
                new ParallelCommandGroup(
                        robot.transfer.kick(),
                        robot.transfer.runTransferForTime(500)
                ),
                robot.intake.setIntakeMotorPow(1),
                robot.intake.flowerStore(),
                new FollowPathCommand(follower, paths.intakeline1()),
                new FollowPathCommand(follower, paths.intakeline2()),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeline3(), .5),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                robot.transfer.runTransferForTime(200)
                        )
                ),
                new WaitCommand(1000),
//                new FollowPathCommand(follower, paths.intakeback()),
//                new FollowPathCommand(follower, paths.intakein(), 0.5),
//                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                robot.intake.setIntakeMotorPow(0)
                        ),
                        new FollowPathCommand(follower, paths.shoot2())
                ),
                robot.intake.flowerUp(),
                robot.vision.checkForTip(),
                new ConditionalCommand(tipRoutine(), noTipRoutine(), robot.vision::isTipConfirmed)
        );
    }

    @Override
    public void initialize() {
        super.reset();
        robot = new Robot(hardwareMap, Robot.OpModeTypes.AUTO, telemetry, Robot.Alliance.RED);

        follower = Constants.create(hardwareMap);
        robot.shooter.configureAim(() -> follower.pose(), robot.goalState);

        robot.intake.flowerStore().schedule();
    }

    @Override
    public void preRun() {
        follower.localizer.setPose(new Pose(58.6, 7.8, Math.toRadians(90)));
        robot.intake.flowerUp().schedule();
        autoRoutine().schedule();
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        PoseStorage.save(follower.pose(), Robot.Alliance.RED);

        telemetry.addData("Tip confirmed", robot.vision.isTipConfirmed());
        telemetry.update();
    }

    @Override
    public void end() {
        PoseStorage.save(follower.pose(), Robot.Alliance.RED);

        robot.intake.setIntakeMotorPow(0).initialize();
        robot.transfer.setTransferPower(0);
        follower.stop();
    }
}
