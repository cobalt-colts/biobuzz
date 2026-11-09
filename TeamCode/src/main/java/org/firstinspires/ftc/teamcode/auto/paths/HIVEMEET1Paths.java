package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class HIVEMEET1Paths {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.5233, 8, 90);
    private final Pose intakeline1 = poseFactory.of(58.5, 12, 90);
    private final Pose intakeline2 = poseFactory.of(9.5, 8, 177.698);
    private final Pose intakeline2Control1 = poseFactory.of(47.8452, 13.8825, 0);
    private final Pose intakeline2Control2 = poseFactory.of(32.7112, 7.0669, 0);
    private final Pose intakeback = poseFactory.of(15, 8.8, -171.7241);
    private final Pose intakein = poseFactory.of(9.5, 8, -171.7241);
    private final Pose shoot2 = poseFactory.of(58.5, 10, 89.4733);
    private final Pose shoot2Control1 = poseFactory.of(24.6426, 8.8522, 0);
    private final Pose shoot2Control2 = poseFactory.of(58.6208, 23.1423, 0);
    private final Pose park = poseFactory.of(15.9593, 94.8356, -1.0159);
    private final Pose parkControl1 = poseFactory.of(58.1334, 25.5177, 0);
    private final Pose parkControl2 = poseFactory.of(14.3187, 37.1689, 0);
    private final Pose parkControl3 = poseFactory.of(2.6758, 95.0712, 0);
    private final Pose park2 = poseFactory.of(9, 94.84, -0.0362);

    public Path intakeline1() {
        return Paths.line(start, intakeline1).constant(intakeline1);
    }

    public Path intakeline2() {
        return Paths.curve(intakeline1, intakeline2Control1, intakeline2Control2, intakeline2).tangent();
    }

    public Path intakeback() {
        return Paths.line(intakeline2, intakeback).reverseTangent();
    }

    public Path intakein() {
        return Paths.line(intakeback, intakein).tangent();
    }

    public Path shoot2() {
        return Paths.curve(intakein, shoot2Control1, shoot2Control2, shoot2).reverseTangent();
    }

    public Path park() {
        return Paths.curve(shoot2, parkControl1, parkControl2, parkControl3, park).tangent();
    }

    public Path park2() {
        return Paths.line(park, park2).reverseTangent();
    }
}
