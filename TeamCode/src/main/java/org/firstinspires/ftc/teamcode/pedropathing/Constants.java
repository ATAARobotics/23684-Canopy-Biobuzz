package org.firstinspires.ftc.teamcode.pedropathing;

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
        c.frontLeftName.set(" frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odoComp");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.716052258108545);
        c.yPodOffset.set(-0.1736801628052719);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2202719446529322);
                Controller secondaryTranslationalForward = Controller.proportional(0.08138459777449746);
                Controller primaryTranslationalLateral = Controller.proportional(0.31329175178980395);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1157529309765481);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0095690587013686));
                c.brake.set(Controller.proportionalFeedforward(0.00813369989616331));

                c.headingFeedback.set(Controller.proportional(4.650083943643753));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.17881558802022274, -0.005374054406345219));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09666738236094859, 0.19512737059580584));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012397788720814437, -5.445777182152711E-4));

                c.maxAchievableForwardVelocity.set(91.56943771038799);
                c.maxAchievableStrafeVelocity.set(78.78967311721264);
                c.naturalForwardDeceleration.set(35.98607472684754);
                c.naturalStrafeDeceleration.set(57.251377442709554);
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