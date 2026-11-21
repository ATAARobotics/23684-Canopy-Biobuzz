package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.OnHardwareThread;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood extends Node {
	private final SafeDevice<Servo> hood;
	public static double STEP = 0.05;

	// Set by auto (or any other caller) to drive the Hood directly. While non-null the gamepad
	// dpad steps are ignored, so auto wins until clearPowerOverride() puts it back in the driver's
	// hands.
	private volatile Double powerOverride;

	public Hood(Orchestrator orchestrator, SafeDevice<Servo> hood) {
		super(orchestrator);
		this.hood = hood;
		orchestrator.getOrCreateTopic("hood/targetPosition", Double.class);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/dpad_up/rising")
	public void increaseAngle() {
		if (powerOverride != null) {
			return;
		}
		double targetPosition = hood.raw().getPosition() + STEP;
		orchestrator.publish("hood/targetPosition", targetPosition);
		// Write raw so it lands on this tick: run() would re-queue through the gate
		// executor, putting an unknown gap between the position read and the write.
		if (!orchestrator.hardware().isRestricted("write")) {
			hood.raw().setPosition(targetPosition);
		}
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/dpad_down/rising")
	public void decreaseAngle() {
		if (powerOverride != null) {
			return;
		}
		double targetPosition = hood.raw().getPosition() - STEP;
		orchestrator.publish("hood/targetPosition", targetPosition);
		if (!orchestrator.hardware().isRestricted("write")) {
			hood.raw().setPosition(targetPosition);
		}
	}

	/**
	 * Move the Hood to an arbitrary servo position and publish it as the target,
	 * so telemetry and the a/b handlers stay in sync.
	 */
	public void moveTo(double target) {
		orchestrator.publish("hood/targetPosition", target);
		hood.labels("write/hood").run(servo -> servo.setPosition(target));
	}

	/**
	 * Drive the Hood from auto: the servo is moved to {@code target} now and the value is latched
	 * so the dpad handlers ignore the driver until {@link #clearPowerOverride()} is called. Pass
	 * {@code null} to release the override.
	 */
	public void setPowerOverride(Double target) {
		powerOverride = target;
		if (target != null) {
			moveTo(target);
		}
	}

	/** Release a {@link #setPowerOverride(Double)} so the dpad controls the Hood again. */
	public void clearPowerOverride() {
		powerOverride = null;
	}

	/** Whether auto currently owns the Hood. */
	public boolean isOverridden() {
		return powerOverride != null;
	}
}
