package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.644625205693282);
        c.yPodOffset.set(-7.555669949749324);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.35845244877412685);
                Controller secondaryTranslationalForward = Controller.proportional(0.13243860179620762);
                Controller primaryTranslationalLateral = Controller.proportional(0.3762191706809934);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1390029308052421);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017212713493015848));
                c.brake.set(Controller.proportionalFeedforward(0.01463080646906347));

                c.headingFeedback.set(Controller.proportional(6.804274242341596));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06008126898415582, 0.005765085392787241));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06917133559092178, 0.04161767215775181));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016590630873239449, 0.001435324595487869));

                c.maxAchievableForwardVelocity.set(60.752487193413);
                c.maxAchievableStrafeVelocity.set(52.86574248985785);
                c.naturalForwardDeceleration.set(76.37940155608862);
                c.naturalStrafeDeceleration.set(66.28249535748823);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}
