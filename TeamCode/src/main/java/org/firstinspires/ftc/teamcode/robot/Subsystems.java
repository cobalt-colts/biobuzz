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
    public Shooter shooter;
    public Transfer transfer;
    public OpModeTypes opmodetype;

    public Subsystems (HardwareMap hardwareMap, OpModeTypes opmodetype) {
        this.drive = new Drive();
        this.intake = new Intake(hardwareMap);
        this.flowerkicker = new FlowerKicker(hardwareMap);
        this.pollenAcquisition = new PollenAcquisition(hardwareMap);
        this.pedro = new Pedro();
        this.shooter = new Shooter(hardwareMap);
        this.transfer = new Transfer(hardwareMap);
        this.opmodetype = opmodetype;
    }
}
