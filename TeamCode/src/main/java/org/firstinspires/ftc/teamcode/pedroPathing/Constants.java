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
				Controller primaryTranslationalForward = Controller.proportional(0.37896964180832216);
				Controller secondaryTranslationalForward = Controller.proportional(0.14001915639284798);
				Controller primaryTranslationalLateral = Controller.proportional(0.6548744924255762);
				Controller secondaryTranslationalLateral = Controller.proportional(0.241958626382537);

				c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
				c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

				c.coast.set(Controller.proportionalFeedforward(0.016011949604487406));
				c.brake.set(Controller.proportionalFeedforward(0.013610157163814295));

				c.headingFeedback.set(Controller.proportional(6.252230412369482));
				c.headingBrakeCoefficients.set(Vector2D.cartesian(0.062298130410495096, 0.006782664649322631));

				c.linearBrakeCoefficients.set(Matrix.diag(0.07443458604818866, 0.044285585881211745));
				c.quadraticBrakeCoefficients.set(Matrix.diag(0.002087965186679848, 0.0027460553523044042));

				c.maxAchievableForwardVelocity.set(68.16974078230814);
				c.maxAchievableStrafeVelocity.set(59.14655998929476);
				c.naturalForwardDeceleration.set(48.39538690084419);
				c.naturalStrafeDeceleration.set(59.25263630067609);
			}
	);

	public static Follower create(SafeHardwareMap h) {
		return new Follower(
				new PinpointLocalizer(h, localizerConfig),
				new Mecanum(h, drivetrainConfig),
				new Foresight(foresightConfig));
	}
}