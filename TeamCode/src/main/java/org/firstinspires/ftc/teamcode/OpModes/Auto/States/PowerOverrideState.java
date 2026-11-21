package org.firstinspires.ftc.teamcode.OpModes.Auto.States;

import com.aaravlabs.autonomy.AbstractState;

/**
 * Drives a subsystem's power override directly (instead of faking a gamepad input) and waits for it to
 * take effect or be released. Replaces the auto's run/stop pairs, which differ only in the override
 * applied, the power topic, and whether the step ends when power rises or falls to zero.
 */
public final class PowerOverrideState extends AbstractState {

	private final AutoContext ctx;
	private final String stateName;
	private final Runnable applyOverride;
	private final String powerTopic;
	private final boolean endWhenStopped;

	private float speed = 0f;

	/**
	 * @param applyOverride  applies the override, e.g. {@code () -> elevator.setPowerOverride(1.0)}
	 * @param powerTopic     topic whose latest value reports the subsystem's actual power
	 * @param endWhenStopped {@code true} ends at {@code |power| == 0}; {@code false} when it rises
	 */
	public PowerOverrideState(
			AutoContext ctx, String stateName, Runnable applyOverride, String powerTopic, boolean endWhenStopped) {
		this.ctx = ctx;
		this.stateName = stateName;
		this.applyOverride = applyOverride;
		this.powerTopic = powerTopic;
		this.endWhenStopped = endWhenStopped;
	}

	@Override
	public void init() {
		applyOverride.run();
		setEndCondition(() -> endWhenStopped ? Math.abs(speed) == 0 : Math.abs(speed) > 0);
	}

	@Override
	public void loop() {
		speed = ctx.orchestrator()
				.getLatestValue(powerTopic, Double.class)
				.orElse(0.0)
				.floatValue();
	}

	@Override
	public String name() {
		return stateName;
	}
}
