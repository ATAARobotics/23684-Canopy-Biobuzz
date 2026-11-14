package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Moves the elevator based on gamepad 2's right stick Y (range -1 to 1). Subscribes directly to
 * {@code g2/right_stick_y} so the gamepad reading lives in the subsystem instead of MainTeleOp's
 * loop. Sets motor power and publishes {@code elevator/power} with the actual power.
 */
public class Elevator extends Node {

	private final SafeDevice<DcMotorEx> elevator;

	public Elevator(Orchestrator orchestrator, SafeDevice<DcMotorEx> elevator) {
		super(orchestrator);
		this.elevator = elevator;
		this.elevator.run(m -> m.setDirection(DcMotorEx.Direction.REVERSE));
	}

	@SubscribedTo(topic = "g2/right_stick_y")
	public void onManualPower(Float stickY) {
		double applied = stickY != null ? -stickY.doubleValue() : 0.0;
		elevator.run(m -> m.setPower(applied));
		orchestrator.publish("elevator/power", applied);
	}

	@Override
	public void close() {
		elevator.run(m -> m.setPower(0.0));
	}
}
