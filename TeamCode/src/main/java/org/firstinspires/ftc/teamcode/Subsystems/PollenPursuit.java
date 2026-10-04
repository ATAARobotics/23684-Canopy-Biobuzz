package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;

/**
 * Approaches the closest blob by driving towards it in the ROBOT's frame, which with a
 * sideways camera means strafing right.
 *
 * <p><b>Camera mounting.</b> The Limelight's optical axis is described by {@link #CAMERA_YAW_DEG},
 * the angle from the robot's forward axis to the camera's forward axis, clockwise positive
 * (towards the robot's right). The camera currently faces straight ahead, so
 * {@code CAMERA_YAW_DEG = 0}: 0 front, 90 right, 180 rear, 270 left. Change this one
 * constant if the mount moves; the geometry below is written to be mount-agnostic.
 *
 * <p><b>Applies to any mount.</b> Velocity is commanded directly at the blob in the robot's
 * own frame, using a sin/cos decomposition rather than a turn-only controller:
 *
 * <pre>
 *   robot bearing  theta = CAMERA_YAW_DEG + cameraBearing      (clockwise from robot nose)
 *   forward       = SPEED * cos(theta)     -> dominant when the blob is ahead
 *   strafe right  = SPEED * sin(theta)     -> takes over as the blob moves off-axis
 *   turn          = TURN_SIGN * TURN_CURVE_GAIN * x^TURN_CURVE_POWER,  x = error past TURN_DEADBAND_DEG
 * </pre>
 *
 * <p>With a forward-facing camera the blob starts near theta=0, so the robot drives forward
 * and turns onto it. As the blob drifts off-axis the forward term falls away and the strafe
 * term picks it up, so a target that moves to one side is still approached directly instead
 * of being chased in a wide arc. The mix is continuous all the way around the compass, so
 * there is no special case for a target behind the robot.
 *
 * <p>The turn term deliberately does NOT null the bearing. It holds the robot at
 * {@link #HOLD_BEARING_DEG} (0 for a forward mount) off the blob, which lines the nose up
 * with the target so the blob stays centred in frame. Setting {@code HOLD_BEARING_DEG} to
 * something else makes the robot approach on an angle while keeping it in view.
 *
 * <p>Inside {@link #TURN_DEADBAND_DEG} of that hold bearing no turn is commanded at all, and
 * beyond it the command follows a power curve up to {@link #MAX_TURN} rather than a straight
 * line. Together they mean the robot settles onto the target instead of sawing back and forth
 * around it; see {@link #TURN_CURVE_POWER} and {@link #TURN_CURVE_GAIN}.
 *
 * <p>Sign conventions: bearings clockwise positive; the {@link Drive} mix uses positive
 * {@link Drive#CMD_STRAFE} for a strafe to the robot's right and positive
 * {@link Drive#CMD_TURN} for a clockwise rotation.
 *
 * <p>There is no target lock and no sequence. This is a steady state: as long as a blob is
 * visible the robot goes for it, and when that blob stops being the largest in frame the
 * tracker hands over to the next one with no interruption. Losing every blob stops the robot.
 *
 * <p>Only active while the driver holds the run button, so nothing moves on its own at
 * OpMode start.
 */
public class PollenPursuit extends Node {

	private static final int HZ = 50;

	/** Angle from the robot's forward axis to the camera's forward axis, clockwise positive. 0 = straight ahead. */
	public static double CAMERA_YAW_DEG = 0.0;

	/**
	 * Sign of the steering correction. Limelight reports the horizontal angle to the target;
	 * if the robot turns AWAY from the blob on first test, flip this to -1.
	 */
	public static double TURN_SIGN = 1.0;

	/**
	 * Robot-frame bearing the controller holds while approaching. 0 for a forward-facing
	 * camera: line the nose up with the blob so it stays centred in frame. Set to e.g. 30 to
	 * approach on an angle instead.
	 */
	public static double HOLD_BEARING_DEG = 0.0;

	/** Mecanum power per degree of bearing-hold error. */
	public static double TURN_GAIN = 0.035;

	/** Ceiling on the turn command. */
	public static double MAX_TURN = 0.32;

	/**
	 * No turn is commanded while the blob sits within this many degrees of
	 * {@link #HOLD_BEARING_DEG}. Stops the robot chasing detection noise and swapping ends
	 * once it is already pointed at the blob.
	 */
	public static double TURN_DEADBAND_DEG = 1.0;

	/**
	 * Exponent on the normalised bearing error, i.e. the shape of the turn curve.
	 *
	 * <p>1 is linear — the old behaviour. 2 tapers the middle of the range so the robot is
	 * gentle as it nears the target and commits to the turn only once the error is large.
	 * 3 and above taper harder. Fractional values are allowed in between.
	 */
	public static double TURN_CURVE_POWER = 2.0;

	/**
	 * Multiplier in front of the curve, so the command is
	 * {@code TURN_CURVE_GAIN * x^TURN_CURVE_POWER} — gain 3 at power 2 gives 3x^2, gain 9 at
	 * power 5 gives 9x^5. 1.0 leaves the curve alone.
	 *
	 * <p>Anything above 1.0 reaches {@link #MAX_TURN} sooner, since that stays a hard ceiling
	 * no matter how the curve is tuned.
	 */
	public static double TURN_CURVE_GAIN = 1.0;

	/** Speed used to close on the blob, before the cosine taper. */
	public static double APPROACH_SPEED = 0.40;

	/**
	 * Speed floor as a fraction of {@link #APPROACH_SPEED} once the robot is past
	 * {@link #HOLD_BEARING_DEG} and has to back up as well as strafe. Keeps the blend from
	 * stalling when cos(theta) crosses zero.
	 */
	public static double MIN_SPEED_FRACTION = 0.25;

	/** Robot-frame bearing republished for telemetry. */
	public static final String BEARING_ROBOT_DEG = "pursuit/bearingRobotDeg";

	/** Signed strafe actually commanded, for telemetry. */
	public static final String STRAFE_CMD = "pursuit/strafeCmd";

	private static final String RUN_BUTTON = "g1/a";
	private static final String ABORT_BUTTON = "g1/b";

	public PollenPursuit(Orchestrator orchestrator) {
		super(orchestrator);
		orchestrator.publish(BEARING_ROBOT_DEG, 0.0);
		orchestrator.publish(STRAFE_CMD, 0.0);
	}

	@RunPeriodically(hz = HZ, hardware = true)
	public void pursue() {
		boolean aborted =
				orchestrator.getLatestValue(ABORT_BUTTON, Boolean.class).orElse(false);
		boolean running = orchestrator.getLatestValue(RUN_BUTTON, Boolean.class).orElse(false);

		if (aborted || !running) {
			publish(0.0, 0.0, 0.0, 0.0);
			return;
		}

		boolean valid =
				orchestrator.getLatestValue(PollenTracker.VALID, Boolean.class).orElse(false);
		if (!valid) {
			publish(0.0, 0.0, 0.0, 0.0);
			return;
		}

		double cameraBearing = orchestrator
				.getLatestValue(PollenTracker.BEARING_DEG, Double.class)
				.orElse(0.0);

		if (!Double.isFinite(cameraBearing)) {
			publish(0.0, 0.0, 0.0, 0.0);
			return;
		}

		// Rotate the camera-frame bearing into the robot frame, wrapped so a mount offset
		// can never push the error past 180 degrees.
		double robotBearing = wrapTo180(CAMERA_YAW_DEG + cameraBearing);

		// Aim at the blob in the robot's own frame. With the camera on the right,
		// robotBearing lands near 90 and this is almost pure right strafe.
		double forward = APPROACH_SPEED * Math.cos(Math.toRadians(robotBearing));
		double strafe = APPROACH_SPEED * Math.sin(Math.toRadians(robotBearing));

		// Hold the camera side toward the blob instead of nulling the bearing.
		double turnError = wrapTo180(robotBearing - HOLD_BEARING_DEG);
		double turn = turnCommand(turnError);

		// Both components share one throttle so forward and strafe never sum past
		// APPROACH_SPEED, and the blend keeps a floor of authority when cos(theta) is ~0.
		double magnitude = Math.hypot(forward, strafe);
		if (magnitude > 1e-6) {
			double floor = MIN_SPEED_FRACTION * APPROACH_SPEED;
			if (magnitude < floor) {
				double scale = floor / magnitude;
				forward *= scale;
				strafe *= scale;
			}
		}

		publish(forward, strafe, turn, robotBearing);
	}

	private void publish(double forward, double strafe, double turn, double robotBearing) {
		orchestrator.publish(Drive.CMD_FORWARD, forward);
		orchestrator.publish(Drive.CMD_STRAFE, strafe);
		orchestrator.publish(Drive.CMD_TURN, turn);
		orchestrator.publish(BEARING_ROBOT_DEG, robotBearing);
		orchestrator.publish(STRAFE_CMD, strafe);
	}

	/**
	 * Turn command for a bearing-hold error: flat zero inside {@link #TURN_DEADBAND_DEG},
	 * then a power curve off to {@link #MAX_TURN}.
	 *
	 * <p>The deadband is subtracted before the curve, which is what lets it start from a
	 * genuine zero: the deadband and the curve share the edge, so there is no step in the
	 * command as the error crosses out of the band.
	 *
	 * <p>At power 1 and gain 1 this reproduces the old linear law exactly, so those two
	 * knobs are a clean baseline to tune away from.
	 */
	private static double turnCommand(double turnError) {
		double excess = Math.abs(turnError) - TURN_DEADBAND_DEG;
		if (excess <= 0.0) {
			return 0.0;
		}

		// Normalise to 0..1 across the bearing error the old linear law saturated at
		// (MAX_TURN / TURN_GAIN, about 9 degrees as tuned), so the power and gain knobs act
		// on a dimensionless x.
		double scale = Math.max(MAX_TURN / TURN_GAIN, 1e-6);
		double x = Math.min(excess / scale, 1.0);

		double magnitude = TURN_CURVE_GAIN * MAX_TURN * Math.pow(x, TURN_CURVE_POWER);

		return clamp(TURN_SIGN * magnitude * Math.copySign(1.0, turnError), -MAX_TURN, MAX_TURN);
	}

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(v, hi));
	}

	/** Normalize an angle to (-180, 180]. */
	private static double wrapTo180(double deg) {
		deg %= 360.0;
		if (deg > 180.0) {
			deg -= 360.0;
		} else if (deg <= -180.0) {
			deg += 360.0;
		}
		return deg;
	}
}
