package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Moves the elevator based on manual power published on {@code elevator/manualPower} (range -1 to 1).
 * When a command arrives, the elevator sets motor power and publishes {@code elevator/power} with the
 * actual power. Publishing the manual command from MainTeleOp and subscribing here lets later
 * subsystems (e.g. a shooter) command the elevator automatically while manual gamepad 2 input
 * overrides it.
 */
public class Elevator extends Node {

	private final SafeDevice<DcMotorEx> elevator;

	public Elevator(Orchestrator orchestrator, SafeDevice<DcMotorEx> elevator) {
		super(orchestrator);
		this.elevator = elevator;
	}

	@SubscribedTo(topic = "elevator/manualPower")
	public void onManualPower(Double power) {
		double applied = power != null ? power : 0.0;
		elevator.run(m -> m.setPower(applied));
		orchestrator.publish("elevator/power", applied);
	}

	@Override
	public void close() {
		elevator.run(m -> m.setPower(0.0));
	}
}
