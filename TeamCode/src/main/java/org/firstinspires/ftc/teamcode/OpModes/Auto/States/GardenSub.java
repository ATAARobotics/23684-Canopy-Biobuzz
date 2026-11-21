package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.safepedropathing.api.Paths;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;

/** The garden route: drive to the garden, shoot if asked, run intake and transfer together, then stop both. */
public final class GardenSub extends Submachine {

	public GardenSub(
			AutoContext ctx,
			Gate gate,
			ShooterKicker shooterKicker,
			Intake intake,
			Elevator elevator,
			boolean shooting) {
		super("Garden", steps(ctx, gate, shooterKicker, intake, elevator, shooting));
	}

	private static List<State> steps(
			AutoContext ctx,
			Gate gate,
			ShooterKicker shooterKicker,
			Intake intake,
			Elevator elevator,
			boolean shooting) {
		List<State> garden = new ArrayList<>();
		garden.add(new FollowPathState(
				ctx,
				"GotoGarden",
				FollowPathState.EndStrategy.SETTLED,
				start -> Paths.line(start, ctx.poses().toGarden).linear(start, ctx.poses().toGarden)));
		if (shooting) {
			garden.add(new ShootState(ctx, gate, shooterKicker, "GardenShoot"));
		}
		garden.add(
				new PowerOverrideState(ctx, "RunIntake", () -> intake.setPowerOverride(1.0), "intake/power", false));
		garden.add(new PowerOverrideState(
				ctx, "RunTransferState", () -> elevator.setPowerOverride(1.0), "elevator/power", false));
		garden.add(new FollowPathState(
				ctx,
				"CollectGarden",
				FollowPathState.EndStrategy.SETTLED,
				start -> Paths.line(start, ctx.poses().pickupGarden).constant(start)));
		garden.add(new PowerOverrideState(
				ctx, "StopTransferState", () -> elevator.setPowerOverride(0.0), "elevator/power", true));
		garden.add(new StopIntakeState(ctx, intake));
		return garden;
	}
}
