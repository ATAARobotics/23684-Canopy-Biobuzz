package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Runs the intake from gamepad 2's left stick Y (range -1 to 1) or, as a convenience, while the
 * left bumper or left trigger is held at {@link #BUMPER_POWER}. When the stick is off-centre it
 * wins, so a centred stick falls through to the bumper/trigger.
 */
public class Intake extends Node {

	/** Power applied while the left bumper is held and the stick is centred. */
	public static double BUMPER_POWER = 1.0;

	// Stick magnitudes inside this band count as centred, so a drifting stick is not treated as
	// an override of the bumper. Applied as a deadzone on the raw reading below.
	private static final double STICK_DEADBAND = 0.05;

	private final SafeDevice<DcMotorEx> intake;
	private final SafeDevice.Scope<DcMotorEx> intakeSet;

	// Written by the stick and bumper callbacks, which Synapse dispatches on separate threads.
	// Each is a single field read back by apply(), so volatile is enough.
	private volatile double manualPower = 0.0;
	private volatile boolean bumperPressed;
	private volatile boolean triggerPressed;

	public Intake(Orchestrator orchestrator, SafeDevice<DcMotorEx> intake) {
		super(orchestrator);
		this.intake = intake;
		this.intakeSet = intake.labels("write/intake").slot("intake/power");
		intake.labels("write/intake").run(m -> m.setDirection(DcMotorSimple.Direction.REVERSE));
	}

	@SubscribedTo(topic = "g2/left_stick_y")
	public void onManualPower(Float stickY) {
		double raw = stickY != null ? -stickY.doubleValue() : 0.0;
		// Deadzone: ignore tiny deflections so stick drift doesn't spin the intake.
		manualPower = Math.abs(raw) < STICK_DEADBAND ? 0.0 : raw;
		apply();
	}

	@SubscribedTo(topic = "g2/left_bumper")
	public void onBumper(Boolean pressed) {
		bumperPressed = Boolean.TRUE.equals(pressed);
		apply();
	}

	@SubscribedTo(topic = "g2/left_trigger_pressed/rising")
	public void onLeftTriggerPressed() {
		triggerPressed = true;
		apply();
	}

	@SubscribedTo(topic = "g2/left_trigger_pressed/falling")
	public void onLeftTriggerReleased() {
		triggerPressed = false;
		apply();
	}

	// Off-centre stick wins; otherwise a held bumper or left trigger runs the intake at full power.
	private void apply() {
		double applied;
		if (Math.abs(manualPower) > STICK_DEADBAND) {
			applied = manualPower;
		} else if (bumperPressed || triggerPressed) {
			applied = BUMPER_POWER;
		} else {
			applied = 0.0;
		}
		intakeSet.run(m -> m.setPower(applied));
		orchestrator.publish("intake/power", applied);
	}

	@Override
	public void close() {
		intakeSet.run(m -> m.setPower(0.0));
	}
}
