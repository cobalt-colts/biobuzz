package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends SubsystemBase {
    MotorEx shooter;

    Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooter = new MotorEx(hardwareMap, "shooter");
        shooter.setInverted(true);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
    }

    @Override
    public void periodic() {
        shooter.setVelocity(1300);
        telemetry.addData("shooter velocity: ", shooter.getVelocity());
    }
}
