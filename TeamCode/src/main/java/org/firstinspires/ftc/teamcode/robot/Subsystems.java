package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.*;
public class Subsystems {
    Drive drive;
    public Subsystems (HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
    }
}
