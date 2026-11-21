package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Moves the elevator based on gamepad 2's right stick Y (range -1 to 1), or while gamepad 2's left
 * bumper is held. Subscribes directly to {@code g2/right_stick_y} and {@code g2/left_bumper} so the
 * gamepad readings live in the subsystem instead of MainTeleOp's loop. Sets motor power and publishes
 * {@code elevator/power} with the actual power. When both inputs are active the stick wins.
 * <p>
 * While the Gate is open and the Shooter is at target RPM the elevator runs itself at
 * {@link #AUTO_POWER}, so pollen feeds without the driver holding the stick. Moving the stick
 * off-centre (or holding the bumper) overrides that until it returns to centre.
 */
public class Elevator extends Node {

	/** Power applied while the Gate is open, the Shooter is at RPM, and the stick is centred. */
	public static double AUTO_POWER = 1.0;

	// Stick magnitudes inside this band count as centred, so a drifting stick is not an override.
	private static final double STICK_DEADBAND = 0.05;

	private final SafeDevice<DcMotorEx> elevator;
	private final SafeDevice.Scope<DcMotorEx> elevatorWrite;

	// Written by the stick, bumper, gate, and Shooter callbacks, which Synapse dispatches on separate
	// threads. Each is a single field read back by apply(), so volatile is enough.
	private volatile double manualPower = 0.0;
	private volatile boolean bumperPressed;
	private volatile boolean gateOpen;
	private volatile boolean shooterAtRpm;

	public Elevator(Orchestrator orchestrator, SafeDevice<DcMotorEx> elevator) {
		super(orchestrator);
		this.elevator = elevator;
		this.elevatorWrite = elevator.labels("write/elevator").slot("elevator/power");
		elevator.labels("write/elevator").run(m -> m.setDirection(DcMotorEx.Direction.REVERSE));
	}

	@SubscribedTo(topic = "g2/right_stick_y")
	public void onManualPower(Float stickY) {
		manualPower = stickY != null ? -stickY.doubleValue() : 0.0;
		apply();
	}

	@SubscribedTo(topic = "g2/left_bumper")
	public void onBumper(Boolean pressed) {
		bumperPressed = Boolean.TRUE.equals(pressed);
		apply();
	}

	@SubscribedTo(topic = "gate/open")
	public void onGateOpen(Boolean open) {
		gateOpen = Boolean.TRUE.equals(open);
		apply();
	}

	@SubscribedTo(topic = "shooter/atRPM")
	public void onAtRpm(Boolean atRpm) {
		shooterAtRpm = Boolean.TRUE.equals(atRpm);
		apply();
	}

	// Off-centre stick wins; otherwise a held bumper runs the elevator, then the elevator feeds on
	// its own while the Gate is open and the Shooter is up to speed.
	private void apply() {
		double applied;
		if (Math.abs(manualPower) > STICK_DEADBAND) {
			applied = manualPower;
		} else if (bumperPressed) {
			applied = AUTO_POWER;
		} else if (gateOpen && shooterAtRpm) {
			applied = AUTO_POWER;
		} else {
			applied = 0.0;
		}
		elevatorWrite.run(m -> m.setPower(applied));
		orchestrator.publish("elevator/power", applied);
	}

	@Override
	public void close() {
		elevatorWrite.run(m -> m.setPower(0.0));
	}
}
