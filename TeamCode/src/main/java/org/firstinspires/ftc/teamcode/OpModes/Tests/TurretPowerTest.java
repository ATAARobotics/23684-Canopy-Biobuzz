package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;

/**
 * Raw power test for the turret: g2's left stick up is positive power, down is negative, and that
 * is the whole OpMode. No controller, no vision, no localization.
 *
 * <p>Start it with the turret facing straight ahead, because that first reading becomes 0 degrees
 * and everything on screen is measured from it.
 *
 * <p>This is where the two sign questions get settled. Push the stick up: the angle should go
 * positive, which is the clockwise direction ({@link TurretConfig#powerSign}). Then watch that the
 * angle follows the way the turret actually moved ({@link TurretConfig#encoderReversed}). Either one
 * backwards means the closed loop in MainTeleOp has positive feedback, so fix it here rather than
 * on the field.
 */
@TeleOp(name = "Turret Power Test", group = "Tests")
public class TurretPowerTest extends SafeOpMode {

	private static final double DEADZONE = 0.05;

	private SafeDevice<CRServo> turretServo;
	private SafeDevice<DcMotorEx> encoder;

	private boolean zeroCaptured;
	private int zeroTicks;
	private int ticks;

	@Override
	protected void onSafeInit() {
		turretServo = safeMap.device(CRServo.class, "turretServo");
		encoder = safeMap.device(DcMotorEx.class, "intake");
	}

	@Override
	protected void onSafeLoop() {
		double power = deadzone(gamepad2.left_stick_y);
		turretServo.run(m -> m.setPower(power));

		ticks = readTicks();
		if (!zeroCaptured) {
			zeroTicks = ticks;
			zeroCaptured = true;
		}
		double sign = TurretConfig.encoderReversed ? -1.0 : 1.0;
		double angleDeg = (ticks - zeroTicks) * sign / TurretConfig.countsPerTurretDegree();

		telemetry.addData("Turret", "power %+.2f   angle %+.1f deg clockwise", power, angleDeg);
		telemetry.addData("Encoder", "%d counts from zero", ticks - zeroTicks);
		telemetry.addData(
				"Signs",
				"powerSign %+.0f   encoderReversed %s",
				TurretConfig.powerSign,
				TurretConfig.encoderReversed
						? "on"
						: "off");
		telemetry.update();
	}

	@Override
	protected void onSafeStop() {
		turretServo.run(m -> m.setPower(0.0));
	}

	private int readTicks() {
		try {
			return encoder.call(DcMotorEx::getCurrentPosition);
		} catch (Exception e) {
			orchestrator.warn("turret encoder read failed: " + e.getMessage());
			return ticks;
		}
	}

	/** Zero out small stick readings so a resting stick does not creep the turret. */
	private static double deadzone(double value) {
		return Math.abs(value) < DEADZONE ? 0.0 : value;
	}
}
