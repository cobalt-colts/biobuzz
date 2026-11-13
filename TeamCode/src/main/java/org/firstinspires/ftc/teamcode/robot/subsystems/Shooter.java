package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.util.GoalState;

@Configurable
public class Shooter extends SubsystemBase {
    public MotorEx shooter;

    Telemetry telemetry;

    public static double kp = 7;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 1.1;

    private InterpLUT shootLUT = new InterpLUT()
            .add(-1, 1230)
            .add(42.3, 1230)
            .add(51.3, 1250)
            .add(53, 1255)
            .createLUT();

    PIDFController shooterPIDF = new PIDFController(kp, ki, kd, kf);

    public static double speed = 0;
    private Supplier<Pose> poseSupplier;
    private GoalState goalState;
    private boolean enabled = true;

    public void configureAim(Supplier<Pose> poseSupplier, GoalState goalState) {
        this.poseSupplier = poseSupplier;
        this.goalState = goalState;
    }

    private double lookupSpeed() {
        if (poseSupplier == null) return 0;
        Pose goal;
        switch (goalState.activeGoal) {
            case FLOWER_RED: goal = Drive.FLOWER_RED_CELL; break;
            case FLOWER_BLUE: goal = Drive.FLOWER_BLUE_CELL; break;
            case NON_FLOWER_BLUE: goal = Drive.NON_FLOWER_BLUE_CELL; break;
            default: goal = Drive.NON_FLOWER_RED_CELL; break;
        }
        Pose pose = poseSupplier.get();
        double distance = Math.hypot(pose.x() - goal.x(), pose.y() - goal.y());
        if (!Double.isFinite(distance)) return 0;
        // Clamp to calibrated distances rather than extrapolating.
        return shootLUT.get(Math.max(0, Math.min(53, distance)));
    }

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooter = new MotorEx(hardwareMap, "shooter");
        shooter.setInverted(true);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
    }

    public Command killShooter() {
        return new InstantCommand(() -> { enabled = false; speed = 0; });
    }

    @Override
    public void periodic() {
        speed = enabled ? lookupSpeed() : 0;
        shooterPIDF.setCoefficients(new PIDFCoefficients(kp, ki, kd, kf));

        double output = shooterPIDF.calculate(shooter.getVelocity(), speed);

        shooter.setVelocity(output);
        telemetry.addData("shooter velocity: ", shooter.getVelocity());
    }
}
