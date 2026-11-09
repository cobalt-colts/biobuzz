package org.firstinspires.ftc.teamcode.robot.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="servo tuner")
public class servotuner extends LinearOpMode {
    Servo gate;

    public static double gatepos = 0.5;

    @Override
    public void runOpMode() {
        gate = hardwareMap.servo.get("flower");

        waitForStart();

        while (opModeIsActive()) {
            gate.setPosition(gatepos);
        }
    }
}
