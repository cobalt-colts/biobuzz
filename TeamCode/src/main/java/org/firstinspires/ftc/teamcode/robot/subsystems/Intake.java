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

    ServoEx leftIntake;
    ServoEx rightIntake;
    ServoExGroup intakeServoGroup;

    public Intake(HardwareMap hardwareMap) {
        intake = new Motor(hardwareMap, "intake");

        leftIntake = new ServoEx(hardwareMap, "leftIntake");
        rightIntake = new ServoEx(hardwareMap, "rightIntake");
        intakeServoGroup = new ServoExGroup(leftIntake, rightIntake);
        rightIntake.setInverted(true);

        intakeServoGroup.set(.8);
        intake.set(0);
    }

    public Command setIntakeServoPos(double pos) {
        return new InstantCommand(() -> intakeServoGroup.set(pos));
    }

    public Command setIntakeMotorPow(double pow) {
        return new InstantCommand(() -> intake.set(pow));
    }
}
