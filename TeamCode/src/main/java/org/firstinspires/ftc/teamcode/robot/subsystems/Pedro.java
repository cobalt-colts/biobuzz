package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Pedro extends SubsystemBase {
    public static class FollowPath extends CommandBase {
        private final Follower follower;
        private final Path path;

        public FollowPath(Follower follower, Path path) {
            this.follower = follower;
            this.path = path;
        }

        @Override
        public void initialize() {
            follower.follow(path);
        }


        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
}
