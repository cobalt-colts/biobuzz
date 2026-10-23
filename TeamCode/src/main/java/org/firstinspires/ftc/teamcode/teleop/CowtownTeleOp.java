package org.firstinspires.ftc.teamcode.teleop;

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

@TeleOp(name="COWTOWN TeleOp")
public class CowtownTeleOp extends CommandOpMode {
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
                .whenPressed(robot.intake.setIntakeMotorPow(-1))
                .whenReleased(robot.intake.setIntakeMotorPow(0));

        // transfer button
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(robot.transfer.setTransferPowerCommand(1))
                .whenInactive(robot.transfer.setTransferPowerCommand(0));

        // change speed +- 50
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(robot.shooter.incrementSpeed(50));

        new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(robot.shooter.incrementSpeed(-50));
    }

    @Override
    public void preRun() {
        follower.localizer.reset();
        // Pinpoint's default reset recalibrates the IMU but preserves its heading.
        follower.setHeading(0);
    }

    @Override
    public void run() {
        // Poll button bindings and run subsystem periodic methods each loop.
        super.run();

        robot.drive.driveFieldCentric(
                driver1.getLeftY(),
                -driver1.getLeftX(),
                -driver1.getRightX(),
                -driver1.getRightY(),
                follower
        );

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
