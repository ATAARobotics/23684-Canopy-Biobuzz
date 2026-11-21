package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.Orchestrator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Auto.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Shared auto state: follower, poses, orchestrator, telemetry, and the pose cursor the next path starts from. */
public final class AutoContext {

	private final Follower follower;
	private final Poses poses;
	private final Orchestrator orchestrator;
	private final Telemetry telemetry;

	// Where the next path starts. Only advanceTo() moves it.
	private Pose lastPose;

	public AutoContext(Follower follower, Poses poses, Orchestrator orchestrator, Telemetry telemetry, Pose lastPose) {
		this.follower = follower;
		this.poses = poses;
		this.orchestrator = orchestrator;
		this.telemetry = telemetry;
		this.lastPose = lastPose;
	}

	public Follower follower() {
		return follower;
	}

	public Poses poses() {
		return poses;
	}

	public Orchestrator orchestrator() {
		return orchestrator;
	}

	public Telemetry telemetry() {
		return telemetry;
	}

	public Pose lastPose() {
		return lastPose;
	}

	public void advanceTo(Pose pose) {
		lastPose = pose;
	}

	/** Whether the follower is slow enough and close enough to its endpoint to call the move done. */
	public boolean settledAtEnd() {
		return Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
				&& follower.distanceToEndpoint() < 4;
	}

	public void logDriveTelemetry() {
		telemetry.addData("Settled?", settledAtEnd());
		telemetry.addData("Velocity", follower.tangentialVelocity());
		telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
		telemetry.addData("Distance to end", follower.distanceToEndpoint());
		Poses.finalPose = follower.pose();
	}
}
