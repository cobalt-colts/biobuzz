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

    public Intake(HardwareMap hardwareMap) {
        intake = new Motor(hardwareMap, "intake");
        intake.set(0);
    }

    public Command setIntakeMotorPow(double pow) {
        return new InstantCommand(() -> intake.set(pow));
    }
}