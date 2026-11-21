package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.AbstractState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

/**
 * Hands the intake back to the gamepad after an auto intake. Kept separate from
 * {@link PowerOverrideState} because it also zeros {@code g2/left_stick_y}, so a held stick does not
 * immediately re-intake.
 */
public final class StopIntakeState extends AbstractState {

	private final AutoContext ctx;
	private final Intake intake;

	private float speed = 0f;

	public StopIntakeState(AutoContext ctx, Intake intake) {
		this.ctx = ctx;
		this.intake = intake;
	}

	@Override
	public void init() {
		intake.clearPowerOverride();
		setEndCondition(() -> Math.abs(speed) == 0);
	}

	@Override
	public void loop() {
		speed = ctx.orchestrator()
				.getLatestValue("intake/power", Double.class)
				.orElse(0.0)
				.floatValue();
		ctx.orchestrator().publish("g2/left_stick_y", 0.0f);
		speed = ctx.orchestrator()
				.getLatestValue("g2/left_stick_y", Float.class)
				.orElse(0.0f);
	}

	@Override
	public String name() {
		return "StopIntake";
	}
}
