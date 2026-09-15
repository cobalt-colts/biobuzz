package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Transfer extends SubsystemBase {
    CRServoEx transfer1;
    CRServoEx transfer2;
    CRServoGroup transferservos;

    ServoEx kicker;

    public Transfer(HardwareMap hardwareMap) {
        transfer1 = new CRServoEx(hardwareMap, "transfer1");
        transfer2 = new CRServoEx(hardwareMap, "transfer2");
        transfer1.setInverted(true);
        transfer2.setInverted(true);
        transferservos = new CRServoGroup(transfer1, transfer2);

        kicker = new ServoEx(hardwareMap, "kicker");
    }

    public Command setTransferServoPower(double pow) {
        return new InstantCommand(() -> transferservos.set(pow));
    }
}
