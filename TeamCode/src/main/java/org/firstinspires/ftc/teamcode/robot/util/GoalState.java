package org.firstinspires.ftc.teamcode.robot.util;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class GoalState {
    public enum Goal {
        NON_FLOWER_RED,
        FLOWER_RED,
        NON_FLOWER_BLUE,
        FLOWER_BLUE
    }

    Robot.Alliance alliance;
    public Goal activeGoal;

    public GoalState(Robot.Alliance robotAlliance) {
        alliance = robotAlliance;
        if (alliance == Robot.Alliance.RED) {
            activeGoal = Goal.NON_FLOWER_RED;
        } else {
            activeGoal = Goal.NON_FLOWER_BLUE;
        }
    }

    public void goalUp() {
        activeGoal = (alliance == Robot.Alliance.RED ? Goal.FLOWER_RED : Goal.FLOWER_BLUE);
    }

    public void goalDown() {
        activeGoal = (alliance == Robot.Alliance.RED ? Goal.NON_FLOWER_RED : Goal.NON_FLOWER_BLUE);
    }
}
