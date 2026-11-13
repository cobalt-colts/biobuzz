package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class HIVEMEET1TIPPaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.516, 8.5032, 90);
    private final Pose shoot = poseFactory.of(58.1134, 120, 93.9474);
    private final Pose shootControl1 = poseFactory.of(61.5747, 69.8402, 0);
    private final Pose shoot2 = poseFactory.of(56, 129, -90);
    private final Pose flower1Start = poseFactory.of(56, 129, -90);
    private final Pose flower1 = poseFactory.of(46, 118, 90);
    private final Pose flower2 = poseFactory.of(46, 128, 90);
    private final Pose shoot3Start = poseFactory.of(46, 128, 90);
    private final Pose shoot3 = poseFactory.of(56, 129, -90);
    private final Pose shoot3Control1 = poseFactory.of(47.9518, 112.3334, 0);
    private final Pose park = poseFactory.of(14, 118, -175.1466);
    private final Pose parkControl1 = poseFactory.of(53.2236, 121.3305, 0);

    public Path shoot() {
        return Paths.curve(start, shootControl1, shoot).tangent();
    }

    public Path shoot2() {
        return Paths.line(shoot, shoot2).constant(shoot2);
    }

    public Path flower1() {
        return Paths.line(flower1Start, flower1).linear(flower1Start, flower1);
    }

    public Path flower2() {
        return Paths.line(flower1, flower2).constant(flower2);
    }

    public Path shoot3() {
        return Paths.curve(shoot3Start, shoot3Control1, shoot3).linear(shoot3Start, shoot3);
    }

    public Path park() {
        return Paths.curve(shoot3, parkControl1, park).tangent();
    }
}
