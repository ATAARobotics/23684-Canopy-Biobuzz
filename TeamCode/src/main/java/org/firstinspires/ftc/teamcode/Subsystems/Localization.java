package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.safepedropathing.localization.Localizer;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.revhub.localizers.PinpointConfig;
import com.aaravlabs.safepedropathing.revhub.localizers.PinpointLocalizer;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;

/**
 * Publishes the robot's field pose from a goBILDA Pinpoint via PedroPathing's {@link
 * PinpointLocalizer} (no {@code Follower}: the drivetrain belongs to the {@code Drive} node in
 * teleop; a future auto can wrap this same localizer in one).
 *
 * <p>The loop is deliberately {@code hardware = false}: {@link PinpointLocalizer#update()} reads
 * the Pinpoint through {@code SafeDevice.call(...)}, which blocks until the hardware thread runs it
 * — calling it from the hardware thread itself would deadlock.
 */
public class Localization extends Node {

	private final Localizer localizer;
	private volatile Pose pose;

	public Localization(Orchestrator orchestrator, SafeHardwareMap safeMap, PinpointConfig config) {
		super(orchestrator);
		this.localizer = new PinpointLocalizer(safeMap, config);
		this.pose = localizer.pose();
	}

	public void setStartPose(Pose startPose) {
		localizer.setPose(startPose);
		pose = startPose;
	}

	@RunPeriodically(hz = 50)
	public void update() {
		localizer.update();
		Pose p = localizer.pose();
		pose = p;

		orchestrator.publish("robot/pose/x", p.x());
		orchestrator.publish("robot/pose/y", p.y());
		// Pose.heading() is RADIANS, CCW-positive (Pedro convention) — convert at the boundary only.
		orchestrator.publish("robot/pose/headingRad", p.heading());
		// Deviation D2: feeds only the vision node's telemetry-only resolved.* geometry. Aiming
		// reads getPose() directly and never consumes this topic.
		orchestrator.publish("vision/robotHeading", Math.toDegrees(p.heading()));
	}

	/** Cached pose (x, y inches; heading radians), safe to read from any thread. */
	public Pose getPose() {
		return pose;
	}
}
