package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;

/**
 * Raw turret power test: g2 left stick up = +power, down = -power, nothing else. Start with the
 * turret facing straight ahead since that first reading becomes 0 degrees for everything.
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
		turretServo = safeMap.device(CRServo.class, "turret");
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
		// Push the stick up: the angle must read positive and follow the turret's real motion. A
		// backwards sign means positive feedback in MainTeleOp.
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

	// Prevent stick drift or slight, accidental movements from moving the turret.
	private static double deadzone(double value) {
		return Math.abs(value) < DEADZONE ? 0.0 : value;
	}
}
