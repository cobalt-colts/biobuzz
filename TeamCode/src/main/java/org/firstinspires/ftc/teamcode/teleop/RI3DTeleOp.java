package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Subsystems;

@TeleOp(name="RI3D TeleOp")
public class RI3DTeleOp extends CommandOpMode {
    Subsystems subsystems;
    GamepadEx driverOp;
    Follower follower;

    @Override
    public void initialize() {
        super.reset();
        subsystems = new Subsystems(hardwareMap, Subsystems.OpModeTypes.TELEOP);

        driverOp = new GamepadEx(gamepad1);
        follower = Constants.create(hardwareMap);

        new GamepadButton(driverOp, GamepadKeys.Button.START)
                .whenPressed(subsystems.drive.recalibratePinpoint(follower));

        new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(subsystems.flowerkicker.setKickerPos(.3))
                .whenReleased(subsystems.flowerkicker.setKickerPos(.85));

        new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(subsystems.intake.setIntakeServoPos(.8));

        new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(subsystems.intake.setIntakeServoPos(.45));

        new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(subsystems.intake.setIntakeMotorPow(-.5))
                .whenReleased(subsystems.intake.setIntakeMotorPow(1));
    }

    @Override
    public void run() {
        subsystems.drive.driveFieldCentric(
                driverOp.getLeftY(),
                -driverOp.getLeftX(),
                -driverOp.getRightX(),
                follower
        );
        follower.update();
        super.run();
    }

    @Override
    public void preRun() {
        subsystems.intake.setIntakeServoPos(.45).schedule();
        subsystems.intake.setIntakeMotorPow(1).schedule();
    }
}
