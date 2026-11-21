package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.AbstractState;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;

/** Spins the shooter up to {@link Shooter#RPM_TARGET}, waits for it to settle, then drops the target. */
public final class ShootState extends AbstractState {

	private static final double SHOOT_SETTLE_SECONDS = 0.25;
	private static final double SHOOT_TIMEOUT_SECONDS = 4.0;

	private final AutoContext ctx;
	private final Gate gate;
	private final ShooterKicker shooterKicker;
	private final String stateName;

	private boolean settled;
	private long settledNanos;

	public ShootState(AutoContext ctx, Gate gate, ShooterKicker shooterKicker, String stateName) {
		this.ctx = ctx;
		this.gate = gate;
		this.shooterKicker = shooterKicker;
		this.stateName = stateName;
	}

	@Override
	public void init() {
		settled = false;
		settledNanos = 0L;
		startTimer(SHOOT_TIMEOUT_SECONDS);
		// Hold the gate open and the kicker at a fixed power so pollen reaches the wheel on spin-up.
		gate.setPowerOverride(true);
		shooterKicker.setPowerOverride(0.6);
		ctx.orchestrator().publish("shooter/targetRPM", Shooter.RPM_TARGET);
		setEndCondition(() -> (settled && elapsedSince(settledNanos) >= SHOOT_SETTLE_SECONDS) || timer().hasElapsed());
	}

	@Override
	public void loop() {
		Boolean atRpm = ctx.orchestrator()
				.getLatestValue("shooter/atRPM", Boolean.class)
				.orElse(false);
		if (!settled && Boolean.TRUE.equals(atRpm)) {
			settled = true;
			settledNanos = System.nanoTime();
		}
		ctx.telemetry()
				.addData(
						stateName,
						"target %.0f   current %.0f   atRPM %s   settled %s",
						Shooter.RPM_TARGET,
						ctx.orchestrator()
								.getLatestValue("shooter/currentRPM", Double.class)
								.orElse(0.0),
						atRpm,
						settled);
	}

	@Override
	public String name() {
		return stateName;
	}

	private static long elapsedSince(long startNanos) {
		return (System.nanoTime() - startNanos) / 1_000_000_000L;
	}
}
