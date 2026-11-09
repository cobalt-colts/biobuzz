package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.concurrent.Delayed;

public class Transfer extends SubsystemBase {
    MotorEx transfer;
    ServoEx kicker;
    Robot robot;

    public static double kickUp = .5425;
    public static double kickDown = .735;


    public Transfer(HardwareMap hardwareMap, Robot mainRobot) {
        transfer = new MotorEx(hardwareMap, "transfer");
        kicker = new ServoEx(hardwareMap,  "flicker");
        robot = mainRobot;
        setTransferPower(0);
        kicker.set(kickDown);
    }

    public void setTransferPower(double pow) {
        transfer.set(pow);
    }

    public InstantCommand setTransferPowerCommand(double pow) {
        return new InstantCommand(() -> setTransferPower(pow));
    }

    public Command kick() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new InstantCommand(() -> Shooter.speed = 1250),
                        new InstantCommand(() -> Shooter.speed = 1260),
                        () -> robot.opmodetype == Robot.OpModeTypes.AUTO
                ),
                new InstantCommand(() -> kicker.set(kickUp)),
                new WaitCommand(250),
                new InstantCommand(() -> kicker.set(kickDown)),
                new InstantCommand(() -> Shooter.speed = 1280)
        );
    }

    public Command setKickPos(double pos) {
        return new InstantCommand(() -> kicker.set(pos));
    }

    public Command runTransferForTime(int time) {
        return new SequentialCommandGroup(
                setTransferPowerCommand(.6),
                robot.intake.setIntakeMotorPow(1),
                new WaitCommand(time),
                robot.intake.setIntakeMotorPow(0),
                setTransferPowerCommand(0)
        );
    }
}
