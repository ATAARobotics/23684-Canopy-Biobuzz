package org.firstinspires.ftc.teamcode.pedroPathing;

import com.aaravlabs.safepedropathing.algorithm.Foresight;
import com.aaravlabs.safepedropathing.algorithm.ForesightConfig;
import com.aaravlabs.safepedropathing.controllers.Controller;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Matrix;
import com.aaravlabs.safepedropathing.math.Pose;
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
	public static final Pose startPose = new Pose(63.0, 9.0, Math.PI / 2.0);



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
		c.name.set("odoComp");
		c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		c.xPodOffset.set(4.183885168841505);
		c.yPodOffset.set(-5.273931308055487);
		c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
		c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
		c.globalDistanceUnit.set(DistanceUnit.INCH);
		c.offsetUnits.set(DistanceUnit.INCH);
	});

	public static ForesightConfig foresightConfig = new ForesightConfig(
			c -> {
				Controller primaryTranslationalForward = Controller.proportional(0.4940601512313794);
				Controller secondaryTranslationalForward = Controller.proportional(0.18254202434961772);
				Controller primaryTranslationalLateral = Controller.proportional(0.7161451839787285);
				Controller secondaryTranslationalLateral = Controller.proportional(0.26459650972839605);

				c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
				c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

				c.coast.set(Controller.proportionalFeedforward(0.013662771259151782));
				c.brake.set(Controller.proportionalFeedforward(0.011613355570279014));

				c.headingFeedback.set(Controller.proportional(6.48011207505562));
				c.headingBrakeCoefficients.set(Vector2D.cartesian(0.058031150882676505, 0.0070460031289883715));

				c.linearBrakeCoefficients.set(Matrix.diag(0.09379014685325165, 0.05640689492957231));
				c.quadraticBrakeCoefficients.set(Matrix.diag(0.002237014120956205, 0.0024576650393864203));

				c.maxAchievableForwardVelocity.set(73.6316567679929);
				c.maxAchievableStrafeVelocity.set(62.568764894505264);
				c.naturalForwardDeceleration.set(39.10667139938933);
				c.naturalStrafeDeceleration.set(62.82531049542589);
			}
	);

	public static Follower create(SafeHardwareMap h) {
		return new Follower(
				new PinpointLocalizer(h, localizerConfig),
				new Mecanum(h, drivetrainConfig),
				new Foresight(foresightConfig));
	}
}