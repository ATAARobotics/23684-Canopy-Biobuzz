package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tuning surface for the turret, exposed in the Panels Configurables dashboard.
 *
 * <p>How Panels Configurables works: every {@code public static} non-final field of a class
 * annotated {@link Configurable} is discovered automatically and can be edited live from the
 * dashboard — no recompiling, no registration call. The semantics that matter here:
 *
 * <ul>
 *   <li>Dashboard → code edits apply immediately. Read the values live on every loop; never copy
 *       them at OpMode init (a copied value will not track the sliders).</li>
 *   <li>Code → dashboard is one-way. Defaults set here do not echo back to the sliders, so do not
 *       expect the UI to follow code-driven changes.</li>
 *   <li>Values live in statics, so they persist across OpMode runs. Great for gains and constants;
 *       wrong for anything that must reset every run.</li>
 * </ul>
 *
 * <p>Because of that last point, the per-run aim offsets ({@code aimOffsetDeg},
 * {@code lateralOffsetCm}) are deliberately <b>not</b> here — they are gamepad-adjusted instance
 * state on the {@code Turret} node, zeroed at OpMode init.
 */
@Configurable
public class TurretConfig {

	// ── Alliance & field ─────────────────────────────────────────────────────────
	/** "RED" or "BLUE". */
	public static String alliance = "RED";
	/** PedroPathing Y of the field midline: audience half below, scoring half above. */
	public static double halfSplitY = 70.5;
	/** HIVE CELL positions in PedroPathing inches (origin at the field corner). */
	public static double cellRedAudienceX = 57.0;
	public static double cellRedAudienceY = 55.0;
	public static double cellRedScoringX = 57.0;
	public static double cellRedScoringY = 86.0;
	public static double cellBlueAudienceX = 84.0;
	public static double cellBlueAudienceY = 55.0;
	public static double cellBlueScoringX = 84.0;
	public static double cellBlueScoringY = 86.0;

	// ── Turret mechanics ─────────────────────────────────────────────────────────
	/** Physical travel range, degrees. Turret zero is forward, CCW-positive. Never command outside. */
	public static double minAngleDeg = -15.0;
	public static double maxAngleDeg = 375.0;
	/** Studica Cypher MAX through-bore encoder, ABI quadrature output (§2.7 of the plan). */
	public static double encoderCountsPerRev = 2048.0;
	/** Turret gear (150T) driven by the servo pinion (60T) and the encoder gear (60T). */
	public static double turretGearTeeth = 150.0;
	/** Encoder gear == servo gear (60T); the encoder shaft turns 2.5× per turret revolution. */
	public static double encoderGearTeeth = 60.0;
	/**
	 * Measurement sign only: a meshed encoder gear turns opposite the turret, so default true. Fix
	 * per bench test — θ must increase as the turret turns CCW. Independent of {@link #powerSign}.
	 */
	public static boolean encoderReversed = true;
	/** Output sign only: +1 = positive servo power turns the turret CCW (left). */
	public static double powerSign = 1.0;
	/** Camera yaw relative to turret zero, degrees, positive left/CCW (cameras not mounted straight). */
	public static double cameraMountYawDeg = 0.0;

	// ── Feedforward (volts; degrees & seconds) ───────────────────────────────────
	public static double ks = 0.0;
	public static double kv = 0.0;
	public static double ka = 0.0;
	public static double maxVelocityDegPerSec = 240.0;
	public static double maxAccelerationDegPerSec2 = 720.0;
	public static double nominalVoltage = 12.0;

	// ── PID (volts per degree / volt·s per degree) ───────────────────────────────
	public static double kp = 0.0;
	public static double ki = 0.0;
	public static double kd = 0.0;
	public static double iZone = Double.POSITIVE_INFINITY;
	public static double integratorMin = -1.0;
	public static double integratorMax = 1.0;
	public static double positionToleranceDeg = 1.0;

	// ── Telemetry ───────────────────────────────────────────────────────────────
	public static boolean verboseTelemetry = false;

	// ── Tuning OpMode step cycle ────────────────────────────────────────────────
	public static double cycleBaseDeg = 0.0;
	public static double cycleTargetDeg = 90.0;
	public static double cycleHalfPeriodSec = 2.0;

	private TurretConfig() {}

	/** Alliance letter used to match cluster names: 'R' or 'B'. Defaults to red. */
	public static char allianceLetter() {
		if (alliance != null && alliance.trim().toUpperCase().startsWith("B")) {
			return 'B';
		}
		return 'R';
	}

	/**
	 * Ticks per turret degree, derived from the mechanical parameters (never entered directly):
	 *
	 * <pre>
	 * countsPerTurretRev     = encoderCountsPerRev × (turretGearTeeth / encoderGearTeeth)
	 *                        = 2048 × (150/60) = 5120 counts / turret rev
	 * countsPerTurretDegree  = countsPerTurretRev / 360 ≈ 14.2222 counts/deg
	 * </pre>
	 */
	public static double countsPerTurretDegree() {
		double rev = encoderCountsPerRev * (turretGearTeeth / Math.max(encoderGearTeeth, 1e-9));
		return rev / 360.0;
	}

	/** Degrees per encoder count: {@code 360 / countsPerTurretRev} ≈ 0.0703125 deg/count. */
	public static double degreesPerCount() {
		return 360.0 / (encoderCountsPerRev * (turretGearTeeth / Math.max(encoderGearTeeth, 1e-9)));
	}
}
