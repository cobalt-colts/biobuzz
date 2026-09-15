package org.firstinspires.ftc.teamcode.robot.tools;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="servo tuner")
public class servotuner extends LinearOpMode {
    Servo gate;

    public static double gatepos = 0;

    @Override
    public void runOpMode() {
        gate = hardwareMap.servo.get("gate");

        waitForStart();

        while (opModeIsActive()) {
            gate.setPosition(gatepos);
        }
    }
}
