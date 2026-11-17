package org.firstinspires.ftc.teamcode.pedro;
import com.google.gson.JsonArray;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lb");
        c.backRightName.set("rb");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("Pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.8842980392335906);
        c.yPodOffset.set(-0.011846555734243919);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2839946178058017);
                Controller secondaryTranslationalForward = Controller.proportional(0.10492842280329702);
                Controller primaryTranslationalLateral = Controller.proportional(0.4070742857284531);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15040307135142);
                c.timeoutConstraint.set(100.0);
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011016974874375906));
                c.brake.set(Controller.proportionalFeedforward(0.00936442864321952));

                c.headingFeedback.set(Controller.proportional(3.9339037925233047));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.058875325128208054, 0.0069607980185640815));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09291190386061339, 0.054622933719402814));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0023686646968431617, 0.002224002590766764));

                c.maxAchievableForwardVelocity.set(76.23000638397635);
                c.maxAchievableStrafeVelocity.set(63.273783362018754);
                c.naturalForwardDeceleration.set(26.27200011725293);
                c.naturalStrafeDeceleration.set(57.254114502548035);
            });
}