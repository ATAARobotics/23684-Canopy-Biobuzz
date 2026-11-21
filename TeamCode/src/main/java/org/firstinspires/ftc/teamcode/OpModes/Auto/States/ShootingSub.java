package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.autonomy.WaitState;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;

/** A full shoot: spin up and fire, feed one through the elevator, then spin the shooter back down. */
public final class ShootingSub extends Submachine {

	public ShootingSub(AutoContext ctx, Gate gate, ShooterKicker shooterKicker, Elevator elevator) {
		super(
				"Shooter",
				Arrays.asList(
						new ShootState(ctx, gate, shooterKicker, "shootingSub"),
						new PowerOverrideState(
								ctx, "RunTransferState", () -> elevator.setPowerOverride(1.0), "elevator/power", false),
						new WaitState(0.2),
						new PowerOverrideState(
								ctx, "StopTransferState", () -> elevator.setPowerOverride(0.0), "elevator/power", true),
						new ShooterStopState(ctx, gate, shooterKicker)));
	}
}
