package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.*;
public class Subsystems {
    public Drive drive;
    public Pedro pedro;
    public Subsystems (HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        pedro = new Pedro();
    }
}
