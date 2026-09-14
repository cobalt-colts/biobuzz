package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class FlowerKicker extends SubsystemBase {
    ServoEx kicker;

    public FlowerKicker(HardwareMap hardwareMap) {
        kicker = new ServoEx(hardwareMap, "flowery");
        kicker.set(.85);
    }

    public Command setKickerPos(double pos) {
        return new InstantCommand(() -> kicker.set(pos));
    }

}
