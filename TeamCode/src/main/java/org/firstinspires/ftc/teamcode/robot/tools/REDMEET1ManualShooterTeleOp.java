package org.firstinspires.ftc.teamcode.robot.tools;

import static org.firstinspires.ftc.teamcode.robot.subsystems.Drive.NON_FLOWER_RED_CELL;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.robot.util.PoseStorage;

@TeleOp(name="TEST Manual Shooter TeleOp")
public class REDMEET1ManualShooterTeleOp extends CommandOpMode {
    Robot robot;

    TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    GamepadEx driver1;
    GamepadEx driver2;

    Follower follower;
    Pose startPose;

    @Override
    public void initialize() {
        super.reset();
        robot = new Robot(hardwareMap, Robot.OpModeTypes.TELEOP, telemetry, Robot.Alliance.RED);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        follower = Constants.create(hardwareMap);
        robot.shooter.configureAim(() -> follower.pose(), robot.goalState);

        // Grab the auto pose during init so the handoff can't expire while waiting for START.
        startPose = PoseStorage.consumeOrDefault(PoseStorage.DEFAULT_RED_TELEOP_POSE, Robot.Alliance.RED);
        follower.setPose(startPose);

        //
        // Driver 1 Controls
        //

        // reset heading button
        new GamepadButton(driver1, GamepadKeys.Button.START)
                .whenPressed(robot.drive.resetHeading(follower));

        // Run intake and transfer together while right bumper is held.
        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        robot.intake.setIntakeMotorPow(1),
                        robot.transfer.setTransferPowerCommand(1)))
                .whenReleased(new SequentialCommandGroup(
                        robot.intake.setIntakeMotorPow(0),
                        robot.transfer.setTransferPowerCommand(0)));

        // invert intake button
        new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(robot.intake.setIntakeMotorPow(-.5))
                .whenReleased(robot.intake.setIntakeMotorPow(0));

        // kick 4th ball
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .toggleWhenActive(new SequentialCommandGroup(
                        robot.transfer.setKickPos(Transfer.kickUp),
                        new WaitCommand(500),
                        robot.transfer.setKickPos(Transfer.kickDown)));

//        // change speed +- 50
//        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP)
//
//        new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN)

        // flower
        new GamepadButton(driver1, GamepadKeys.Button.CIRCLE)
                .whenPressed(robot.intake.flowerUp());

        new GamepadButton(driver1, GamepadKeys.Button.SQUARE)
                .whenPressed(robot.intake.flowerDown());

        // GoalSnap
        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> robot.drive.goalsnap = true)
                .whenReleased(() -> robot.drive.goalsnap = false);

        //
        // Driver 2 Controls
        //

        // Goal State
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.goalState.goalUp());

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> robot.goalState.goalDown());

    }

    @Override
    public void preRun() {
        // Shooter speed follows the distance lookup table.

        // Re-apply at start in case the localizer (Pinpoint) finished resetting after init.
        follower.setPose(startPose);

        robot.intake.flowerUp().schedule();
    }

    @Override
    public void run() {
        driver1.gamepad = PanelsGamepad.INSTANCE.getFirstManager()
                .asCombinedFTCGamepad(gamepad1);
        driver2.gamepad = PanelsGamepad.INSTANCE.getSecondManager()
                .asCombinedFTCGamepad(gamepad2);
        driver1.gamepad.start = gamepad1.start || driver1.gamepad.options;
        driver2.gamepad.start = gamepad2.start || driver2.gamepad.options;

        super.run();

        robot.drive.driveFieldCentric(
                driver1.getLeftY(),
                -driver1.getLeftX(),
                -driver1.getRightX(),
                -driver1.getRightY(),
                follower,
                robot.goalState.activeGoal
        );

        follower.update();

        panelsTelemetry.addData("shooter velocity", robot.shooter.shooter.getVelocity());
        panelsTelemetry.addData("shooter target velocity", Shooter.speed);
        panelsTelemetry.addData("active goal", robot.goalState.activeGoal);
        telemetry.addData("active goal", robot.goalState.activeGoal);
        telemetry.addData("distance to red goal", Math.hypot(
                follower.pose().x() - NON_FLOWER_RED_CELL.x(),
                follower.pose().y() - NON_FLOWER_RED_CELL.y()));

        telemetry.update();
        panelsTelemetry.update();
    }

    @Override
    public void end() {
        robot.intake.setIntakeMotorPow(0).initialize();
        robot.transfer.setTransferPower(0);
        follower.stop();
    }
}
