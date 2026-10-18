package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

/** Turret tunables, edited live from the Panels Configurables dashboard; read them every loop. */
@Configurable
public class TurretConfig {

	// ── Alliance & field ─────────────────────────────────────────────────────────

	// Turning this on aims from the camera; off uses odometry only.
	public static boolean aimFromVision = true;

	// "RED" or "BLUE".
	public static String alliance = "RED";

	// Y of the field midline in inches; above is the scoring half, below the audience half.
	public static double halfSplitY = 70.5;

	// Hive cell positions, PedroPathing inches from the field corner; used only when the camera cannot see the cell.
	public static double cellRedAudienceX = 57.0;
	public static double cellRedAudienceY = 55.0;
	public static double cellRedScoringX = 57.0;
	public static double cellRedScoringY = 86.0;
	public static double cellBlueAudienceX = 84.0;
	public static double cellBlueAudienceY = 55.0;
	public static double cellBlueScoringX = 84.0;
	public static double cellBlueScoringY = 86.0;

	// ── Turret mechanics ─────────────────────────────────────────────────────────

	// Travel limits, degrees clockwise from straight ahead; power cuts at either end (negative = left).
	public static double minAngleDeg = -1.0;
	public static double maxAngleDeg = 290.0;

	// Studica Cypher MAX through-bore encoder, on the encoder gear shaft.
	public static double encoderCountsPerRev = 2048.0;

	// Gear tooth counts; the encoder gear shares the servo pinion, so these must match the hardware.
	public static double turretGearTeeth = 150.0;

	public static double encoderGearTeeth = 60.0;

	// Bench check (measurement only): true if the encoder counts backwards as the turret turns clockwise.
	public static boolean encoderReversed = false;

	// Output check: +1 if positive power turns clockwise, -1 if counter-clockwise; wrong sign = runaway.
	public static double powerSign = -1.0;

	// Camera yaw relative to the turret, degrees clockwise; 0 = straight ahead.
	public static double cameraMountYawDeg = 0.0;

	// ── Feedforward (volts, degrees, seconds) ─────────────────────────────────────

	public static double ks = 0.9;
	public static double kv = 0.045;
	public static double ka = 0.0;

	// Trapezoid profile limits, degrees/s and degrees/s².
	public static double maxVelocityDegPerSec = 240.0;
	public static double maxAccelerationDegPerSec2 = 720.0;

	// Fallback when the battery voltage read fails.
	public static double nominalVoltage = 12.0;

	// ── PID (volts) ───────────────────────────────────────────────────────────────

	public static double kp = 0.215;
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

	// True adds the extra turret control and vision telemetry.
	public static boolean verboseTelemetry = true;

	// ── Tuning OpMode step cycle ──────────────────────────────────────────────────

	public static double cycleBaseDeg = -1.0;
	public static double cycleTargetDeg = 90.0;
	public static double cycleHalfPeriodSec = 2.0;

	private TurretConfig() {}

	/** Alliance letter for cluster names: 'R' for red, 'B' for blue. */
	public static char allianceLetter() {
		if (alliance != null && alliance.trim().toUpperCase().startsWith("B")) {
			return 'B';
		}
		return 'R';
	}

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
