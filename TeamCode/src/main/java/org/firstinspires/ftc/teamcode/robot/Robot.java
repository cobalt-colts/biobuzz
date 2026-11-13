package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.robot.util.GoalState;


public class Robot {
    public enum OpModeTypes {
        TELEOP,
        AUTO
    }

    public enum Alliance {
        RED,
        BLUE
    }

    public Drive drive;
    public Intake intake;
    public FlowerKicker flowerkicker;
    // public PollenAcquisition pollenAcquisition;
    public Shooter shooter;
    public Transfer transfer;
    public OpModeTypes opmodetype;
    public GoalState goalState;
    public Vision vision;

    public Robot (HardwareMap hardwareMap, OpModeTypes opmodetype, Telemetry telemetry, Alliance alliance) {
        if (opmodetype == OpModeTypes.TELEOP) {
            this.drive = new Drive(alliance);
        }
        this.intake = new Intake(hardwareMap, opmodetype);
//        this.flowerkicker = new FlowerKicker(hardwareMap);
//        this.pollenAcquisition = new PollenAcquisition(hardwareMap);
        this.shooter = new Shooter(hardwareMap, telemetry);
        this.transfer = new Transfer(hardwareMap, this);
        this.opmodetype = opmodetype;
        this.goalState = new GoalState(alliance);
        this.vision = new Vision(hardwareMap, alliance);
    }
}
