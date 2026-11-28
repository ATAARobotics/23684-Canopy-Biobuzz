package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

/** Turret tunables, edited live from the Panels Configurables dashboard; read them every loop. */
@Configurable
public class TurretConfig {

	// ── Field ────────────────────────────────────────────────────────────────────

	// Y of the field midline in inches; above is the scoring half, below the audience half.
	public static double halfSplitY = FieldMap.MIDLINE;

	// HIVE cell anchors live in FieldMap as surveyed CAD positions in PedroPathing inches

	// ── Turret mechanics ─────────────────────────────────────────────────────────

	// Travel limits, degrees clockwise from straight ahead; power cuts at either end (negative = left).
	public static double minAngleDeg = -90;
	public static double maxAngleDeg = 180;

	// Studica Cypher MAX through-bore encoder, on the encoder gear shaft.
	public static double encoderCountsPerRev = 8192.0;

	// Gear tooth counts; the encoder gear shares the servo pinion, so these must match the hardware.
	public static double turretGearTeeth = 150.0;

	public static double encoderGearTeeth = 60.0;

	public static double encoderZeroTicks = 0.0;

	// True when the encoder counts backwards as the turret turns clockwise; flips the measured angle sign.
	// Matches the staging config, which turns the turret in the right direction.
	public static boolean encoderReversed = true;

	public static double offsetX = -1.1023622;
	public static double offsetY = -0.62992126;

	public static boolean enabledByDefault = true;

	// ── Static friction feedforward (volts) ──────────────────────────────────────

	// Voltage added in the direction of the goal to break static friction; same role as
	// SimpleMotorFeedforward.ks but without the velocity/acceleration terms.
	public static double ks = 0.9;

	// Fallback when the battery voltage read fails.
	public static double nominalVoltage = 12.0;

	// ── PID (volts) ───────────────────────────────────────────────────────────────

	public static double kp = 0.04;
	public static double ki = 0.0;
	public static double kd = 0.0;

	// Integral acts only within this many degrees of the goal; Infinity disables the check.
	public static double iZone = Double.POSITIVE_INFINITY;

	// Bounds on what the integral term alone may contribute, in volts.
	public static double integratorMin = -1.0;
	public static double integratorMax = 1.0;

	// How close counts as on target, degrees.
	public static double positionToleranceDeg = 1.0;

	// ── Telemetry ─────────────────────────────────────────────────────────────────

	// True adds the extra turret control telemetry.
	public static boolean verboseTelemetry = true;

	// ── Tuning OpMode step cycle ──────────────────────────────────────────────────

	public static double cycleBaseDeg = 0.0;
	public static double cycleTargetDeg = 90.0;
	public static double cycleHalfPeriodSec = 2.0;

	private TurretConfig() {}

	/** Encoder counts per turret degree, derived from the gear tooth counts above. */
	public static double countsPerTurretDegree() {
		double rev = encoderCountsPerRev * (turretGearTeeth / Math.max(encoderGearTeeth, 1e-9));
		return rev / 360.0;
	}

	/** Degrees per encoder count; the inverse of the above conversion. */
	public static double degreesPerCount() {
		return 360.0 / (encoderCountsPerRev * (turretGearTeeth / Math.max(encoderGearTeeth, 1e-9)));
	}
}
