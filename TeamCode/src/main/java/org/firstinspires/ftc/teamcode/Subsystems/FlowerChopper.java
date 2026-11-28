package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.OnHardwareThread;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class FlowerChopper extends Node {
	private final SafeDevice<Servo> flowerChopper;
	public static double CLOSED_POSITION = 0.3;
	public static double OPEN_POSITION = 0.66;
	private  double position = CLOSED_POSITION;

	boolean isFlowerButtonPressed = false;

	public FlowerChopper(Orchestrator orchestrator, SafeDevice<Servo> flowerChopper) {
		super(orchestrator);
		this.flowerChopper = flowerChopper;
		orchestrator.getOrCreateTopic("flowerChopper/position", Double.class);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/left_bumper/rising")
	public void flowerIntake() {
		synchronized (FlowerChopper.class) {
			position =  position == OPEN_POSITION  ? CLOSED_POSITION : OPEN_POSITION;
		}
		flowerChopper.raw().setPosition(position);
		orchestrator.publish("flowerChopper/position", position);
	}
}
