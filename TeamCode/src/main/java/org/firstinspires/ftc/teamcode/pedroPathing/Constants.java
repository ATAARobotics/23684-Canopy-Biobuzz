package org.firstinspires.ftc.teamcode.pedroPathing;

import com.aaravlabs.safepedropathing.algorithm.Foresight;
import com.aaravlabs.safepedropathing.algorithm.ForesightConfig;
import com.aaravlabs.safepedropathing.controllers.Controller;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Matrix;
import com.aaravlabs.safepedropathing.math.Vector2D;
import com.aaravlabs.safepedropathing.revhub.drivetrains.Mecanum;
import com.aaravlabs.safepedropathing.revhub.drivetrains.MecanumConfig;
import com.aaravlabs.safepedropathing.revhub.localizers.PinpointConfig;
import com.aaravlabs.safepedropathing.revhub.localizers.PinpointLocalizer;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;
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
                Controller primaryTranslationalForward = Controller.proportional(0.23210000797185648);
                Controller secondaryTranslationalForward = Controller.proportional(0.0857547511191673);
                Controller primaryTranslationalLateral = Controller.proportional(0.3360340335383034);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12415559639765843);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010324461266630176));
                c.brake.set(Controller.proportionalFeedforward(0.00877579207663565));

                c.headingFeedback.set(Controller.proportional(3.9742079831565356));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05250118782327535, 0.0090366621533246));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08380740225228185, 0.06946605948229298));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016040022196118508, 0.001948141066986833));

                c.maxAchievableForwardVelocity.set(90.42771856860752);
                c.maxAchievableStrafeVelocity.set(45.856385211775525);
                c.naturalForwardDeceleration.set(34.11863151483826);
                c.naturalStrafeDeceleration.set(62.085867207625064);
            }
    );
        public static Follower create(SafeHardwareMap h) {
            return new Follower(
                    new PinpointLocalizer(h, localizerConfig),
                    new Mecanum(h, drivetrainConfig),
                    new Foresight(foresightConfig)
            );
        }
}