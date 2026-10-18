package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tuning surface for the turret, shown in the Panels Configurables dashboard.
 *
 * <p>How Panels Configurables works: every {@code public static} non-final field of a class
 * annotated {@link Configurable} is discovered automatically and can be edited live from the
 * dashboard — no recompiling, no registration call. Three things to remember:
 *
 * <ul>
 *   <li>Dashboard → code applies immediately. Read the values live on every loop; never copy them
 *       at OpMode init, or they stop tracking the sliders.
 *   <li>Code → dashboard is one-way. Defaults set here do not echo back to the sliders.
 *   <li>Values live in statics, so they persist across OpMode runs. Good for gains and constants,
 *       wrong for anything that must reset every run — which is why the per-run aim and lateral
 *       offsets live on the {@code Turret} node instead.
 * </ul>
 */
@Configurable
public class TurretConfig {

	// ── Alliance & field ─────────────────────────────────────────────────────────

	/**
	 * Whether the turret is allowed to aim from the camera. Off means odometry only: every aim
	 * comes from the cell's fixed position plus the robot's pose, and the camera keeps streaming
	 * and reporting so the two can still be compared. Separate from the turret's own on/off,
	 * which is the x button on the gamepad.
	 */
	public static boolean aimFromVision = true;

	/** Alliance to aim for: "RED" or "BLUE". */
	public static String alliance = "RED";

	/** Y of the field midline. Above it is the scoring half, below it the audience half. */
	public static double halfSplitY = 70.5;

	// Hive cell positions in PedroPathing inches, origin at the field corner. Only used when the
	// camera cannot see the cell.
	public static double cellRedAudienceX = 57.0;
	public static double cellRedAudienceY = 55.0;
	public static double cellRedScoringX = 57.0;
	public static double cellRedScoringY = 86.0;
	public static double cellBlueAudienceX = 84.0;
	public static double cellBlueAudienceY = 55.0;
	public static double cellBlueScoringX = 84.0;
	public static double cellBlueScoringY = 86.0;

	// ── Turret mechanics ─────────────────────────────────────────────────────────

	/**
	 * Travel range in degrees, clockwise from straight ahead. The turret cuts power rather than
	 * push past either end. Negative is to the left and positive to the right, so −1 is just past
	 * straight ahead on the left and +290 is the far right.
	 */
	public static double minAngleDeg = -1.0;
	public static double maxAngleDeg = 290.0;

	/** Studica Cypher MAX through-bore encoder on the encoder gear shaft. */
	public static double encoderCountsPerRev = 2048.0;

	/** Teeth on the turret gear. */
	public static double turretGearTeeth = 150.0;

	/** Teeth on the encoder gear. It shares the servo pinion, so these two match. */
	public static double encoderGearTeeth = 60.0;

	/**
	 * Bench check for measurement only: does the angle increase as the turret turns clockwise? True
	 * means the encoder counts the other way and the ticks are flipped before use. Nothing to do
	 * with {@link #powerSign}.
	 */
	public static boolean encoderReversed = false;

	/**
	 * Bench check for output only: +1 when positive servo power turns the turret clockwise, −1
	 * when it turns counter-clockwise. Together with {@link #encoderReversed} this decides whether
	 * the loop has negative or positive feedback, so getting it wrong means the turret runs away.
	 */
	public static double powerSign = -1.0;

	/** Camera yaw relative to the turret, degrees clockwise. Zero when it points straight ahead. */
	public static double cameraMountYawDeg = 0.0;

	// ── Feedforward (volts, degrees, seconds) ─────────────────────────────────────

	public static double ks = 0.9;
	public static double kv = 0.045;
	public static double ka = 0.0;

	/** Trapezoid profile limits on the turret's motion, degrees/s and degrees/s². */
	public static double maxVelocityDegPerSec = 240.0;
	public static double maxAccelerationDegPerSec2 = 720.0;

	/** Used when the battery voltage read fails. */
	public static double nominalVoltage = 12.0;

	// ── PID (volts) ───────────────────────────────────────────────────────────────

	public static double kp = 0.215;
	public static double ki = 0.0;
	public static double kd = 0.0;

	/** The integral only acts within this many degrees of the goal; ∞ disables the check. */
	public static double iZone = Double.POSITIVE_INFINITY;

	/** Bounds on what the integral term alone may contribute, in volts. */
	public static double integratorMin = -1.0;
	public static double integratorMax = 1.0;

	/** How close counts as on target, degrees. */
	public static double positionToleranceDeg = 1.0;

	// ── Telemetry ─────────────────────────────────────────────────────────────────

	public static boolean verboseTelemetry = true;

	// ── Tuning OpMode step cycle ──────────────────────────────────────────────────

	public static double cycleBaseDeg = -1.0;
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
	 * Encoder counts per turret degree, derived from the mechanical parameters (never entered
	 * directly):
	 *
	 * <pre>
	 * countsPerTurretRev    = encoderCountsPerRev × (turretGearTeeth / encoderGearTeeth)
	 *                       = 2048 × (150/60) = 5120 counts / turret rev
	 * countsPerTurretDegree = countsPerTurretRev / 360 ≈ 14.2222 counts/deg
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
