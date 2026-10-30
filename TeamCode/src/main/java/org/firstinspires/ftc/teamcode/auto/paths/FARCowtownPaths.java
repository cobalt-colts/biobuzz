package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class FARCowtownPaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(132.5, 59, 90);
    private final Pose moveForward = poseFactory.of(125, 59, 180);
    private final Pose park = poseFactory.of(43.6604, 14.9844, -176.3943);
    private final Pose parkControl1 = poseFactory.of(111.5762, 19.0677, 0);

    public Path moveForward() {
        return line(start, moveForward).constant(moveForward);
    }

    public Path park() {
        return curve(moveForward, parkControl1, park).tangent();
    }
}
