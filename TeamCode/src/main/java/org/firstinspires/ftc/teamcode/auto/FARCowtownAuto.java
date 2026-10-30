package org.firstinspires.ftc.teamcode.auto;

import android.os.DeadObjectException;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.auto.paths.FARCowtownPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@Autonomous(name="FAR COWTOWN Auto")
public class FARCowtownAuto extends CommandOpMode {
    Robot robot;
    Follower follower;

    FARCowtownPaths paths = new FARCowtownPaths();

    Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitCommand(4000),
                robot.transfer.runTransferForTime(1000),
                new WaitCommand(1000),
                robot.transfer.runTransferForTime(1000),
                new WaitCommand(1000),
                robot.transfer.runTransferForTime(1000),
                new WaitCommand(1000),
                robot.transfer.runTransferForTime(1000),
                new WaitCommand(1000),
                robot.shooter.killShooter(),
                new FollowPathCommand(follower, paths.moveForward()),
                new FollowPathCommand(follower, paths.park(), true),
                new TurnToCommand(follower, Math.toRadians(90))
        );
    }

    @Override
    public void initialize() {
        super.reset();
        robot = new Robot(hardwareMap, Robot.OpModeTypes.AUTO, telemetry);

        follower = Constants.create(hardwareMap);

        robot.intake.retractFlower().schedule();
    }

    @Override
    public void preRun() {
        Shooter.speed = 1365;
        follower.localizer.setPose(new Pose(132.5,59,Math.toRadians(180)));
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
        robot.intake.setIntakeMotorPow(0).initialize();
        robot.transfer.setTransferPower(0);
        follower.stop();
    }
}
