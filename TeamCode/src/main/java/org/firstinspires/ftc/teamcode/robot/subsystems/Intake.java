package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

public class Intake extends SubsystemBase {
    Motor intake;
    ServoEx flower;

    public static double flowerUp = .3;
    public static double flowerDrive = .32;
    public static double flowerDown = .4;

    public Intake(HardwareMap hardwareMap) {
        intake = new Motor(hardwareMap, "intake");
        intake.setInverted(true);
        flower = new ServoEx(hardwareMap, "flower");
        intake.set(0);
    }

    public Command setIntakeMotorPow(double pow) {
        return new InstantCommand(() -> intake.set(pow));
    }

    public Command flowerDown() {
        return new InstantCommand(() -> flower.set(flowerDown));
    }

    public Command flowerUp() {
        return new InstantCommand(() -> flower.set(flowerDrive));
    }

    public Command flowerStore() {
        return new InstantCommand(() -> flower.set(flowerUp));
    }
}