package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A servo-actuated barrier mounted at the top of the Elevator, directly below the Shooter opening.
 * The Gate controls whether pollen can travel up from the Elevator into the Shooter.
 * <p>
 * Position 0.0 closes the Gate to block pollen from passing into the Shooter.
 * Position 1.0 opens the Gate so pollen can pass into the Shooter when the Elevator is powered.
 * <p>
 * Gamepad2 A opens the Gate and gamepad2 B closes it, via Synapse rising-edge subscriptions.
 */
public class Gate extends Node {

	private static final double CLOSED_POSITION = 0.0;
	private static final double OPEN_POSITION = 1.0;

	// Last commanded position, used for telemetry. open()/close() submit to the hardware thread
	// asynchronously, so this tracks intent instead of the servo's actual feedback.
	private volatile double position = CLOSED_POSITION;

	private final SafeDevice<Servo> gate;

	public Gate(Orchestrator orchestrator, SafeDevice<Servo> gate) {
		super(orchestrator);
		this.gate = gate;
	}

	/** Close the Gate: servo position 0.0 blocks pollen from reaching the Shooter. */
	@SubscribedTo(topic = "g2/b/rising")
	public void close() {
		position = CLOSED_POSITION;
		gate.run(servo -> servo.setPosition(CLOSED_POSITION));
	}

	/** Open the Gate: servo position 1.0 clears the path so pollen can enter the Shooter. */
	@SubscribedTo(topic = "g2/a/rising")
	public void open() {
		position = OPEN_POSITION;
		gate.run(servo -> servo.setPosition(OPEN_POSITION));
	}

	/** Whether the Gate was last commanded open. */
	public boolean isOpen() {
		return position == OPEN_POSITION;
	}
}
