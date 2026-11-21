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

	public Hood(Orchestrator orchestrator, SafeDevice<Servo> hood) {
		super(orchestrator);
		this.hood = hood;
		orchestrator.getOrCreateTopic("hood/targetPosition", Double.class);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/dpad_up/rising")
	public void increaseAngle() {
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
		double targetPosition = hood.raw().getPosition() - STEP;
		orchestrator.publish("hood/targetPosition", targetPosition);
		if (!orchestrator.hardware().isRestricted("write")) {
			hood.raw().setPosition(targetPosition);
		}
	}
}
