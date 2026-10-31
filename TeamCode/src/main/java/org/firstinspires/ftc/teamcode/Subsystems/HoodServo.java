package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServo extends Node {
	private final SafeDevice<Servo> hoodServo;

	public HoodServo(Orchestrator orchestrator, SafeDevice<Servo> hoodServo) {
		super(orchestrator);
		this.hoodServo = hoodServo;
		hoodServo.run(m -> m.setDirection(Servo.Direction.REVERSE));
		orchestrator.getOrCreateTopic("servo/pos", Double.class);
	}

	@RunPeriodically(hz = 50)
	public void Update() {
		double angle = orchestrator.getLatestValue("servo/pos", Double.class).orElse(0.0);
		hoodServo.run((servo -> servo.setPosition(angle)));
	}

	//    @SubscribedTo(topic = "g1/a/rising")
	//    public void increaseAngle() {
	//        hoodServo.run(m -> m.setPosition(m.getPosition() + 0.05));
	//    }
	//    @SubscribedTo(topic = "g1/b/falling")
	//    public void decreaseAngle() {
	//        hoodServo.run(m -> m.setPosition(m.getPosition() - 0.05));
	//    }
}
