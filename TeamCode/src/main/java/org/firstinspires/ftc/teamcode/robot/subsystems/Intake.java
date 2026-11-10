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
    ServoEx flower1;
    ServoEx flower2;
    ServoExGroup flower;

    public static double flowerUp = 0.0;
    public static double flowerDrive = .1;
    public static double flowerDown = .48;

    public Intake(HardwareMap hardwareMap) {
        intake = new Motor(hardwareMap, "intake");
        intake.setInverted(true);

        flower1 = new ServoEx(hardwareMap, "flower1");
        flower2 = new ServoEx(hardwareMap, "flower2").setInverted(true);
        flower = new ServoExGroup(flower1, flower2);

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