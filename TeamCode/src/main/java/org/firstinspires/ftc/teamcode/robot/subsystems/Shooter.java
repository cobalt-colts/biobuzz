package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Shooter extends SubsystemBase {
    public MotorEx shooter;

    Telemetry telemetry;

    public static double kp = 2.5;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 1.2;

    PIDFController shooterPIDF = new PIDFController(kp, ki, kd, kf);

    public static int speed = 1300;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooter = new MotorEx(hardwareMap, "shooter");
        shooter.setInverted(true);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
    }

    public Command incrementSpeed(int changeBy) {
        return new InstantCommand(() -> speed += changeBy);
    }

    public Command killShooter() {
        return new InstantCommand(() -> speed = 0);
    }

    @Override
    public void periodic() {
        shooterPIDF.setCoefficients(new PIDFCoefficients(kp, ki, kd, kf));

        double output = shooterPIDF.calculate(shooter.getVelocity(), speed);

        shooter.setVelocity(output);
        telemetry.addData("shooter velocity: ", shooter.getVelocity());
    }
}
