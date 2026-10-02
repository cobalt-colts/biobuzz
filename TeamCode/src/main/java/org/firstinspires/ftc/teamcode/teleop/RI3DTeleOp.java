package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="RI3D TeleOp")
public class RI3DTeleOp extends CommandOpMode {
    Robot robot;
    GamepadEx driverOp;
    Follower follower;

    @Override
    public void initialize() {
        super.reset();
        robot = new Robot(hardwareMap, Robot.OpModeTypes.TELEOP, telemetry);

        driverOp = new GamepadEx(gamepad1);
        follower = Constants.create(hardwareMap);
        robot.pollenAcquisition.start();

        new GamepadButton(driverOp, GamepadKeys.Button.START)
                .whenPressed(robot.drive.resetHeading(follower));

//        new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(robot.flowerkicker.setKickerPos(.3))
//                .whenReleased(robot.flowerkicker.setKickerPos(.85));

        /*new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(robot.intake.setIntakeServoPos(.7));

        new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(robot.intake.setIntakeServoPos(.485));

        new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(robot.intake.setIntakeMotorPow(-.5))
                .whenReleased(robot.intake.setIntakeMotorPow(1));
*/
        new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(robot.transfer.setTransferServoPower(1))
                .whenInactive(robot.transfer.setTransferServoPower(0));

        Command acquirePollen = robot.pollenAcquisition.acquire(follower)
                /*.alongWith(
                        robot.intake.setIntakeServoPos(.45),
                        robot.intake.setIntakeMotorPow(1)
                );*/;

        // Hold X to turn and approach the selected cluster; release to resume stick drive.
        new GamepadButton(driverOp, GamepadKeys.Button.X)
                .whenHeld(acquirePollen);
    }

    @Override
    public void run() {
        super.run();

        if (!robot.pollenAcquisition.isActive()) {
            robot.drive.driveFieldCentric(
                    driverOp.getLeftY(),
                    -driverOp.getLeftX(),
                    -driverOp.getRightX(),
                    -driverOp.getRightY(),
                    follower
            );
        }

        follower.update();

        telemetry.addData("Pollen assist", robot.pollenAcquisition.isActive());
        telemetry.addData("Pollen target", robot.pollenAcquisition.hasTarget());
        robot.pollenAcquisition.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void preRun() {
//        robot.intake.setIntakeServoPos(.485).schedule();
//        robot.intake.setIntakeMotorPow(1).schedule();
        follower.localizer.reset();
    }

    @Override
    public void end() {
        robot.pollenAcquisition.stop();
        follower.stop();
    }
}
