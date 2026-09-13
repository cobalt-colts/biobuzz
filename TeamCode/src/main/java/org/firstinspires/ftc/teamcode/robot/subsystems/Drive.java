package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class Drive extends SubsystemBase {
    private final MecanumDrive mecanum;

    private GoBildaPinpointDriver pinpoint;
    public double heading;

    public Drive(HardwareMap hardwareMap) {

        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void periodic() {
        super.periodic();

        pinpoint.update();

        heading = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void driveFieldCentric(double strafe, double forward, double turn) {
        mecanum.driveFieldCentric(strafe, forward, turn, heading);
    }

    public Command recalibratePinpoint = new InstantCommand(() -> pinpoint.recalibrateIMU());

    public void stop() {
        mecanum.stop();
    }
}
