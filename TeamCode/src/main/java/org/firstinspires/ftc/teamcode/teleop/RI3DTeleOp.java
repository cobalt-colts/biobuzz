package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
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
        subsystems.pollenAcquisition.start();

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

        Command acquirePollen = subsystems.pollenAcquisition.acquire(follower)
                .alongWith(
                        subsystems.intake.setIntakeServoPos(.45),
                        subsystems.intake.setIntakeMotorPow(1)
                );

        new GamepadButton(driverOp, GamepadKeys.Button.X)
                .whenHeld(acquirePollen);
    }

    @Override
    public void run() {
        super.run();

        if (!subsystems.pollenAcquisition.isActive()) {
            subsystems.drive.driveFieldCentric(
                    driverOp.getLeftY(),
                    -driverOp.getLeftX(),
                    -driverOp.getRightX(),
                    follower
            );
        }

        follower.update();

        telemetry.addData("Pollen assist", subsystems.pollenAcquisition.isActive());
        telemetry.addData("Pollen target", subsystems.pollenAcquisition.hasTarget());
        telemetry.addData("Pollen entry", "%.2f, %.2f",
                subsystems.pollenAcquisition.getEntryX(),
                subsystems.pollenAcquisition.getEntryY());
        telemetry.update();
    }

    @Override
    public void preRun() {
        subsystems.intake.setIntakeServoPos(.45).schedule();
        subsystems.intake.setIntakeMotorPow(1).schedule();
    }

    @Override
    public void end() {
        subsystems.pollenAcquisition.stop();
        follower.stop();
    }
}
