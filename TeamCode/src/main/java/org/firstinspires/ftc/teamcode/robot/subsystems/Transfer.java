package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Transfer extends SubsystemBase {
    MotorEx transfer;
    ServoEx kicker;

    public Transfer(HardwareMap hardwareMap) {
        transfer = new MotorEx(hardwareMap, "transfer");
        setTransferPower(0);
    }

    public void setTransferPower(int pow) {
        transfer.set(pow);
    }

    public InstantCommand setTransferPowerCommand(int pow) {
        return new InstantCommand(() -> setTransferPower(pow));
    }
}
