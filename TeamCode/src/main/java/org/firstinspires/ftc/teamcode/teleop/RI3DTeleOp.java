package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.robot.Subsystems;

@TeleOp(name="RI3D TeleOp")
public class RI3DTeleOp extends CommandOpMode {
    Subsystems subsystems = new Subsystems(hardwareMap);
    GamepadEx driverOp;

    @Override
    public void initialize() {
        super.reset();

        driverOp = new GamepadEx(gamepad1);
    }
    @Override
    public void run() {
        subsystems.drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX()
        );
        super.run();
    }
}
