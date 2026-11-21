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
		c.xPodOffset.set(4.829470116322435);
		c.yPodOffset.set(-5.5639468215581935);
		c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
		c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
		c.globalDistanceUnit.set(DistanceUnit.INCH);
		c.offsetUnits.set(DistanceUnit.INCH);
	});

	public static ForesightConfig foresightConfig = new ForesightConfig(
			c -> {
				Controller primaryTranslationalForward = Controller.proportional(0.4594043952197504);
				Controller secondaryTranslationalForward = Controller.proportional(0.1697376485221762);
				Controller primaryTranslationalLateral = Controller.proportional(0.6814278476234191);
				Controller secondaryTranslationalLateral = Controller.proportional(0.25176938160942175);

				c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
				c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

				c.coast.set(Controller.proportionalFeedforward(0.012702986304365005));
				c.brake.set(Controller.proportionalFeedforward(0.010797538358710253));

				c.headingFeedback.set(Controller.proportional(6.14006385986772));
				c.headingBrakeCoefficients.set(Vector2D.cartesian(0.059106445035615254, 0.006367657660320755));

				c.linearBrakeCoefficients.set(Matrix.diag(0.07367740985171523, 0.0732403293704071));
				c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025444181688659123, 0.0017805572006910472));

				c.maxAchievableForwardVelocity.set(70.13587146967937);
				c.maxAchievableStrafeVelocity.set(61.96428931165135);
				c.naturalForwardDeceleration.set(40.07346222246295);
				c.naturalStrafeDeceleration.set(72.67321620264734);
			}
	);

	public static Follower create(SafeHardwareMap h) {
		return new Follower(
				new PinpointLocalizer(h, localizerConfig),
				new Mecanum(h, drivetrainConfig),
				new Foresight(foresightConfig));
	}
}