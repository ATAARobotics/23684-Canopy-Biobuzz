package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Owns the full PedroPathing {@link Follower} — localizer, drivetrain and Foresight together —
 * and publishes the robot's field pose.
 *
 * <p>This replaces the old {@code Localization} node, which held only a localizer and left the
 * drivetrain to the {@code Drive} node. Having the whole follower means autos can run straight
 * off this instance instead of building a second one, and it keeps a single localizer per OpMode:
 * two localizers on the same Pinpoint would each integrate odometry every loop and double the
 * apparent speed.
 *
 * <p><b>In teleop the follower is odometry only.</b> Nothing here calls {@code follow}, {@code
 * hold} or {@code manual}, so the {@code Mecanum} never writes motor power and {@code Drive} stays
 * the only thing driving the wheels. The drivetrain is still constructed, and its constructor
 * still sets motor directions and zero-power behavior on all four motors, so both of those must
 * agree with {@code Drive} (see {@link Constants#drivetrainConfig}).
 *
 * <p>The loop is deliberately {@code hardware = false}: {@link Follower#update()} reads the
 * Pinpoint through {@code SafeDevice.call(...)}, which blocks until the hardware thread runs it —
 * calling it from the hardware thread itself would deadlock.
 */
public class Pedro extends Node {

	private final Follower follower;
	private volatile Pose pose;

	public Pedro(Orchestrator orchestrator, SafeHardwareMap safeMap) {
		super(orchestrator);
		this.follower = Constants.create(safeMap);
		this.pose = follower.pose();
	}

	/** The follower itself, for consumers that want paths or raw odometry. */
	public Follower follower() {
		return follower;
	}

	/**
	 * Tells the follower where the robot actually is, replacing whatever odometry believes.
	 *
	 * <p>Call once from OpMode init — the Pinpoint's encoder counters start wherever the last run
	 * left them, so the first pose is only as good as the robot's placement on the field.
	 *
	 * <p>Deliberately never called from {@link #update()} or any other periodic loop: the write
	 * queues onto the hardware thread, so re-seeding it every cycle would both fight {@link
	 * #update()} for that thread and overwrite the odometry the loop just read.
	 *
	 * @param startPose field position, x and y in Pedro inches, heading in CCW-positive radians
	 */
	public void setStartPose(Pose startPose) {
		follower.setPose(startPose);
		pose = startPose; // Seed the cache now: the next update() republishes it to subscribers.
	}

	@RunPeriodically(hz = 50)
	public void update() {
		follower.update();
		Pose p = follower.pose();
		pose = p;

		orchestrator.publish("robot/pose/x", p.x());
		orchestrator.publish("robot/pose/y", p.y());
		// Pose.heading() is RADIANS, CCW-positive (Pedro convention) — convert at the boundary only.
		orchestrator.publish("robot/pose/headingRad", p.heading());
		// Deviation D2: feeds only the vision node's telemetry-only resolved.* geometry. Aiming
		// reads pose() directly and never consumes this topic.
		orchestrator.publish("vision/robotHeading", Math.toDegrees(p.heading()));
	}

	/** Cached pose (x, y inches; heading radians), safe to read from any thread. */
	public Pose getPose() {
		return pose;
	}
}
