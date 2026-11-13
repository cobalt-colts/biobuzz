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
    private final Pose intakeline2 = poseFactory.of(20, 7.5, -177.9197);
    private final Pose intakeline2Control1 = poseFactory.of(47.9458, 19.3171, 0);
    private final Pose intakeline2Control2 = poseFactory.of(33.0131, 7.9727, 0);
    private final Pose intakeline3 = poseFactory.of(9.5, 7.5, 180);
    private final Pose shoot2 = poseFactory.of(58.5, 10, 89.4733);
    private final Pose shoot2Control1 = poseFactory.of(24.6426, 8.8522, 0);
    private final Pose shoot2Control2 = poseFactory.of(58.6208, 23.1423, 0);
    private final Pose flower = poseFactory.of(25, 48.8994, 167.5757);
    private final Pose flowerControl1 = poseFactory.of(56.1206, 29.5433, 0);
    private final Pose flowerControl2 = poseFactory.of(40.787, 45.4214, 0);
    private final Pose flower2 = poseFactory.of(12.5, 48.8994, 180);
    private final Pose park = poseFactory.of(9, 94.84, 0.4459);
    private final Pose parkControl1 = poseFactory.of(42.0906, 48.9928, 0);
    private final Pose parkControl2 = poseFactory.of(24.3739, 94.9596, 0);

    public Path intakeline1() {
        return Paths.line(start, intakeline1).constant(intakeline1);
    }

    public Path intakeline2() {
        return Paths.curve(intakeline1, intakeline2Control1, intakeline2Control2, intakeline2).tangent();
    }

    public Path intakeline3() {
        return Paths.line(intakeline2, intakeline3).tangent();
    }

    public Path shoot2() {
        return Paths.curve(intakeline3, shoot2Control1, shoot2Control2, shoot2).reverseTangent();
    }

    public Path flower() {
        return Paths.curve(shoot2, flowerControl1, flowerControl2, flower).tangent();
    }

    public Path flower2() {
        return Paths.line(flower, flower2).tangent();
    }

    public Path park() {
        return Paths.curve(flower2, parkControl1, parkControl2, park).reverseTangent();
    }
}
