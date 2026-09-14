package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.*;


public class Subsystems {
    public enum OpModeTypes {
        TELEOP,
        AUTO
    }

    public Drive drive;
    public Pedro pedro;
    public Intake intake;
    public FlowerKicker flowerkicker;
    public PollenAcquisition pollenAcquisition;
    public OpModeTypes opmodetype;

    public Subsystems (HardwareMap hardwareMap, OpModeTypes opmodetype) {
        this.drive = new Drive();
        this.intake = new Intake(hardwareMap);
        this.flowerkicker = new FlowerKicker(hardwareMap);
        this.pollenAcquisition = new PollenAcquisition(hardwareMap);
        this.pedro = new Pedro();
        this.opmodetype = opmodetype;
    }
}
