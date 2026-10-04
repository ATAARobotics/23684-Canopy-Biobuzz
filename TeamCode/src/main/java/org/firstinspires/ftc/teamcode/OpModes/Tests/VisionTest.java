package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilTagVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.PointOfInterest;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionCameraPose;

/**
 * Standalone bench test for {@link AprilTagVision}. Does not involve the drivetrain, so the robot
 * can sit still while you aim it at the field and dial in the numbers.
 *
 * <p>Alliance colour is {@link AprilTagVision#DEFAULT_ALLIANCE_LETTER} in the vision subsystem.
 * Change it there before a match rather than at the Driver Station, because SDK 12.0's {@code OpMode}
 * exposes no match-manifest alliance accessor to read at runtime.
 *
 * <p>Gamepad:
 *
 * <ul>
 *   <li>{@code left_stick_x} — hold to sweep the turret yaw
 *   <li>{@code dpad_left} / {@code dpad_right} — step turret yaw 1 degree
 *   <li>{@code dpad_up} / {@code dpad_down} — step decimation
 *   <li>{@code a} — toggle the point of interest
 *   <li>{@code b} — toggle verbose telemetry
 *   <li>{@code right_bumper} — reset camera extrinsics to defaults
 * </ul>
 */
@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends SafeOpMode {

	/** Turret yaw sweep rate while the stick is deflected, degrees per second. */
	private static final double YAW_RATE_DEG_PER_SEC = 60.0;

	/** Stick deflection below this is treated as centred, to avoid stick drift. */
	private static final double STICK_DEADBAND = 0.05;

	private VisionCameraPose cameraPose;
	private PointOfInterest poi;
	private AprilTagVision vision;

	/** Mirrors the node's verbose flag so the B button can flip it. */
	private boolean verbose;

	@Override
	protected void onSafeInit() {
		cameraPose = new VisionCameraPose();
		// Disabled to start. Flip it on once the plain cluster numbers look sane.
		poi = new PointOfInterest("SCORE", PointOfInterest.Frame.FIELD);

		GamepadAdaptor.attach(orchestrator, gamepad1, "g1");

		vision = new AprilTagVision(
				orchestrator,
				hardwareMap,
				cameraPose,
				poi,
				AprilTagVision.DEFAULT_ALLIANCE_LETTER,
				AprilTagVision.DEFAULT_CAMERA_NAME);
		orchestrator.registerNode("vision", vision);
		vision.setVerbose(verbose);
	}

	@Override
	protected void onSafeLoop() {
		handleControls();
		vision.publishTelemetry(telemetry);
		telemetry.update();
	}

	/** Map gamepad input onto the vision command topics. */
	private void handleControls() {
		float stick = orchestrator.getLatestValue("g1/left_stick_x", Float.class).orElse(0.0f);
		if (Math.abs(stick) > STICK_DEADBAND) {
			// Convert stick deflection to a yaw rate, then to this frame's increment.
			double nudge = stick * YAW_RATE_DEG_PER_SEC / AprilTagVision.POLL_HZ;
			orchestrator.publish("vision/nudgeCameraYaw", nudge);
		}

		if (rising("g1/dpad_left")) {
			orchestrator.publish("vision/nudgeCameraYaw", -1.0);
		}
		if (rising("g1/dpad_right")) {
			orchestrator.publish("vision/nudgeCameraYaw", 1.0);
		}

		if (rising("g1/dpad_up")) {
			orchestrator.publish("vision/setDecimation", currentDecimation() + 1);
		}
		if (rising("g1/dpad_down")) {
			orchestrator.publish("vision/setDecimation", currentDecimation() - 1);
		}

		if (rising("g1/a")) {
			poi.setEnabled(!poi.isEnabled());
		}

		if (rising("g1/b")) {
			verbose = !verbose;
			vision.setVerbose(verbose);
		}

		if (rising("g1/right_bumper")) {
			orchestrator.publish("vision/resetCameraPose", true);
		}
	}

	/** True only on the poll where a boolean topic transitions from false to true. */
	private boolean rising(String topic) {
		return orchestrator.getLatestValue(topic + "/rising", Boolean.class).orElse(false);
	}

	private int currentDecimation() {
		return orchestrator.getLatestValue("vision/decimation", Integer.class).orElse(AprilTagVision.DEFAULT_DECIMATION);
	}
}