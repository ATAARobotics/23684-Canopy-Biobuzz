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
 *   turn          = TURN_SIGN * TURN_GAIN * (theta - HOLD_BEARING_DEG)
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
	public static double MAX_TURN = 0.40;

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
		double turn = clamp(TURN_SIGN * TURN_GAIN * turnError, -MAX_TURN, MAX_TURN);

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
