package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.safepedropathing.api.Paths;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.FollowPathState.EndStrategy;

/** Drive to the audience-half flower, pause to let the intake grab it, then leave. */
public final class FlowerAudienceSub extends Submachine {

	public FlowerAudienceSub(AutoContext ctx) {
		super(
				"Flower",
				Arrays.asList(
						new FollowPathState(
								ctx,
								"ToAudienceFlowerState",
								EndStrategy.NOT_FOLLOWING,
								start -> start.y() > 72
										? Paths.line(start, ctx.poses().audienceFlowerIntake)
												.constant(Math.toRadians(90))
										: Paths.curve(start, ctx.poses().firstFlowerControl1, ctx.poses().audienceFlowerIntake)
												.linear(start, ctx.poses().audienceFlowerIntake)),
						new WaitState(1),
						new FollowPathState(
								ctx,
								"LeaveAudienceFlowerState",
								EndStrategy.NOT_FOLLOWING,
								start -> Paths.line(start, ctx.poses().audienceFlowerLeave)
										.constant(Math.toRadians(90)))));
	}
}
