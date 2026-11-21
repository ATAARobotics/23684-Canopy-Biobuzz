package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.safepedropathing.api.Paths;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.FollowPathState.EndStrategy;

/** Drive to the scoring-half flower, pause to let the intake grab it, then leave. */
public final class FlowerScoringSub extends Submachine {

	public FlowerScoringSub(AutoContext ctx) {
		super(
				"Flower",
				Arrays.asList(
						new FollowPathState(
								ctx,
								"ToScoringFlowerState",
								EndStrategy.NOT_FOLLOWING,
								start -> start.y() > 72
										? Paths.line(start, ctx.poses().scoringFlowerIntake)
												.constant(Math.toRadians(90))
										: Paths.curve(start, ctx.poses().firstFlowerControl1, ctx.poses().scoringFlowerIntake)
												.linear(start, ctx.poses().scoringFlowerIntake)),
						new WaitState(1),
						new FollowPathState(
								ctx,
								"LeaveScoringFlowerState",
								EndStrategy.NOT_FOLLOWING,
								start -> Paths.line(start, ctx.poses().scoringFlowerLeave)
										.constant(Math.toRadians(90)))));
	}
}
