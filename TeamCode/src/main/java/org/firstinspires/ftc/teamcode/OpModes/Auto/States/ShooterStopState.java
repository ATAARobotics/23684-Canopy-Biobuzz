package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.AbstractState;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;

/** Spins the shooter back down after a shoot: closes the gate and hands the kicker back to the Shooter. */
public final class ShooterStopState extends AbstractState {

	private final AutoContext ctx;
	private final Gate gate;
	private final ShooterKicker shooterKicker;

	private double speed = 0;

	public ShooterStopState(AutoContext ctx, Gate gate, ShooterKicker shooterKicker) {
		this.ctx = ctx;
		this.gate = gate;
		this.shooterKicker = shooterKicker;
	}

	@Override
	public void init() {
		gate.setPowerOverride(false);
		shooterKicker.clearPowerOverride();
		setEndCondition(() -> Math.abs(speed) == 0);
	}

	@Override
	public void loop() {
		ctx.orchestrator().publish("shooter/targetRPM", 0.0);
		speed = ctx.orchestrator()
				.getLatestValue("shooter/targetRPM", Double.class)
				.orElse(0.0);
	}

	@Override
	public String name() {
		return "ShooterStop";
	}
}
