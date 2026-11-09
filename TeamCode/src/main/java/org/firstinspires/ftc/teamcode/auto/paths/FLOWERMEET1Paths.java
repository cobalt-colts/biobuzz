package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class FLOWERMEET1Paths {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.516, 133.1963, 90);
    private final Pose shootNpark = poseFactory.of(11.3158, 122.0473, -179.6455);
    private final Pose shootNparkControl1 = poseFactory.of(58.6008, 122.3399, 0);
    private final Pose movethefuckoutoftheway = poseFactory.of(11.32, 90, 0);

    public Path shootNpark() {
        return Paths.curve(start, shootNparkControl1, shootNpark).tangent();
    }

    public Path movethefuckoutoftheway() {
        return Paths.line(shootNpark, movethefuckoutoftheway).constant(movethefuckoutoftheway);
    }
}
