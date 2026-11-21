package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.AbstractState;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;

/**
 * Drives to a pose: builds a path from {@link AutoContext#lastPose()}, follows it, then advances the
 * cursor to the pose the path ended at. Reading the cursor from the path itself is what keeps it in
 * step with where the robot actually went.
 */
public final class FollowPathState extends AbstractState {

	/** How a driving state decides it has arrived. */
	public enum EndStrategy {
		AT_PARAMETRIC_END,
		SETTLED,
		NOT_FOLLOWING
	}

	/** Builds the path from the pose the robot is at when the state starts. */
	@FunctionalInterface
	public interface PathFactory {
		Path build(Pose startPose);
	}

	private final AutoContext ctx;
	private final String stateName;
	private final EndStrategy strategy;
	private final PathFactory pathFactory;

	private Path path;

	public FollowPathState(AutoContext ctx, String stateName, EndStrategy strategy, PathFactory pathFactory) {
		this.ctx = ctx;
		this.stateName = stateName;
		this.strategy = strategy;
		this.pathFactory = pathFactory;
	}

	@Override
	public void init() {
		path = pathFactory.build(ctx.lastPose());
		switch (strategy) {
			case AT_PARAMETRIC_END:
				setEndCondition(() -> ctx.follower().atParametricEnd());
				break;
			case SETTLED:
				setEndCondition(ctx::settledAtEnd);
				break;
			case NOT_FOLLOWING:
				setEndCondition(() -> !ctx.follower().following());
				break;
		}
		ctx.follower().follow(path);
	}

	@Override
	public void loop() {
		ctx.follower().update();
		ctx.logDriveTelemetry();
	}

	@Override
	public void stop() {
		ctx.advanceTo(path.endPose());
		ctx.follower().stop();
	}

	@Override
	public String name() {
		return stateName;
	}
}
