package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Shooter extends SubsystemBase {
    MotorEx shooter;

    public Shooter(HardwareMap hardwareMap) {
        shooter = new MotorEx(hardwareMap, "shooter");
        shooter.setInverted(true);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
    }

    @Override
    public void periodic() {
        shooter.setVelocity(1300);
    }
}
