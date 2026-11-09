package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@TeleOp(name="MEET 1 TeleOp")
public class MEET1TeleOp extends CommandOpMode {
    Robot robot;

    TelemetryManager.TelemetryWrapper panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    GamepadEx driver1;
    GamepadEx driver2;

    Follower follower;

    @Override
    public void initialize() {
        super.reset();
        robot = new Robot(hardwareMap, Robot.OpModeTypes.TELEOP, telemetry);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        follower = Constants.create(hardwareMap);

        // reset heading button
        new GamepadButton(driver1, GamepadKeys.Button.START)
                .whenPressed(robot.drive.resetHeading(follower));

        // intake button
        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(robot.intake.setIntakeMotorPow(1))
                .whenReleased(robot.intake.setIntakeMotorPow(0));

        // invert intake button
        new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(robot.intake.setIntakeMotorPow(-.5))
                .whenReleased(robot.intake.setIntakeMotorPow(0));

        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .toggleWhenActive(robot.transfer.kick());

        // change speed +- 50
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(robot.shooter.incrementSpeed(50));

        new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(robot.shooter.incrementSpeed(-50));

        new GamepadButton(driver1, GamepadKeys.Button.CIRCLE)
                .whenPressed(robot.intake.flowerUp());

        new GamepadButton(driver1, GamepadKeys.Button.SQUARE)
                .whenPressed(robot.intake.flowerDown());
    }

    @Override
    public void preRun() {
        Shooter.speed = 1280;

        follower.localizer.reset();
        // Pinpoint's default reset recalibrates the IMU but preserves its heading.
        follower.setHeading(0);

        robot.intake.flowerUp().schedule();
    }

    @Override
    public void run() {
        // Refresh Panels and physical controller input before polling button bindings.
        driver1.gamepad = PanelsGamepad.INSTANCE.getFirstManager()
                .asCombinedFTCGamepad(gamepad1);
        driver2.gamepad = PanelsGamepad.INSTANCE.getSecondManager()
                .asCombinedFTCGamepad(gamepad2);
        // Panels supplies Options but does not populate the FTC Start alias.
        driver1.gamepad.start = gamepad1.start || driver1.gamepad.options;
        driver2.gamepad.start = gamepad2.start || driver2.gamepad.options;

        // Poll button bindings and run subsystem periodic methods each loop.
        super.run();

        robot.drive.driveFieldCentric(
                driver1.getLeftY(),
                -driver1.getLeftX(),
                -driver1.getRightX(),
                -driver1.getRightY(),
                follower
        );

        robot.transfer.setTransferPower(Math.sqrt(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        follower.update();

        // Panels adds the colon; graph parsing requires "label: number".
        panelsTelemetry.addData("shooter velocity", robot.shooter.shooter.getVelocity());
        panelsTelemetry.addData("shooter target velocity", Shooter.speed);

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
