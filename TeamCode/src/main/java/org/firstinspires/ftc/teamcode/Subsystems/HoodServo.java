package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.OnHardwareThread;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServo extends Node {
	private final SafeDevice<Servo> hoodServo;
	public static double STEP = 0.05;

	public HoodServo(Orchestrator orchestrator, SafeDevice<Servo> hoodServo) {
		super(orchestrator);
		this.hoodServo = hoodServo;
		orchestrator.getOrCreateTopic("hood/targetPosition", Double.class);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g1/a/rising")
	public void increaseAngle() {
		double higherPosition = hoodServo.raw().getPosition() + STEP;
		hoodServo.raw().setPosition(higherPosition);
	}

	@SubscribedTo(topic = "g1/b/rising")
	public void decreaseAngle() {
		double lowerPosition = hoodServo.raw().getPosition() - STEP;
		hoodServo.raw().setPosition(lowerPosition);
	}
}
