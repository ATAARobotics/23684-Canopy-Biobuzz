package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Owns the whole Pedro follower (localizer, drivetrain, Foresight) and publishes the field pose. */
public class Pedro extends Node {

	// The constructor sets motor directions and zero-power behavior on all four motors, so its
	// config must agree with the Drive node.
	private final Follower follower;
	// Pose the g2 A button restores, or null when this node was built without one.
	private final Pose resetPose;
	private volatile Pose pose;

	public Pedro(Orchestrator orchestrator, SafeHardwareMap safeMap) {
		this(orchestrator, safeMap, null);
	}

	/**
	 * @param resetPose pose that g2 A restores, or null to leave the button unbound
	 */
	public Pedro(Orchestrator orchestrator, SafeHardwareMap safeMap, Pose resetPose) {
		super(orchestrator);
		this.follower = Constants.create(safeMap);
		this.resetPose = resetPose;
		this.pose = follower.pose();
	}

	/** g2 A: put the robot back at the reset pose. */
	@SubscribedTo(topic = "g2/a/rising")
	public void g2ResetPose() {
		if (resetPose != null) {
			setStartPose(resetPose);
		}
	}

	/** The follower that build paths or read raw odometry. */
	public Follower follower() {
		return follower;
	}

	/** Sets the field pose. Call from OpMode init only, never from a periodic loop.
	 * @param startPose x, y in inches, heading in CCW-positive radians */
	public void setStartPose(Pose startPose) {
		follower.setPose(startPose);
		pose = startPose;
	}

	@RunPeriodically(hz = 50)
	public void update() {
		follower.update();
		Pose p = follower.pose();
		pose = p;

		orchestrator.publish("robot/pose/x", p.x());
		orchestrator.publish("robot/pose/y", p.y());
		// Heading is CCW-positive radians (Pedro convention). Converted to degrees only at publish.
		orchestrator.publish("robot/pose/headingRad", follower.pose().heading());
		orchestrator.publish("vision/robotHeading", Math.toDegrees(p.heading()));
	}

	/** Cached pose. Safe to read from any thread. */
	public Pose getPose() {
		return pose;
	}
}
