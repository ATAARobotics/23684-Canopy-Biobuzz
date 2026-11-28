package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Moves the elevator based on gamepad 2's right stick Y (range -1 to 1), or while gamepad 2's left
 * bumper or left trigger is held. Subscribes directly to {@code g2/right_stick_y},
 * {@code g2/left_bumper}, and {@code g2/left_trigger_pressed} so the gamepad readings live in the
 * subsystem instead of MainTeleOp's loop. Sets motor power and publishes {@code elevator/power} with
 * the actual power. When the stick is off-centre it wins.
 * <p>
 * While the Gate is open and the Shooter is at target RPM the elevator runs itself at
 * {@link #AUTO_POWER}, so pollen feeds without the driver holding the stick. Moving the stick
 * off-centre (or holding the bumper or trigger) overrides that until it returns to centre.
 * <p>
 * Auto drives the elevator through {@link #setPowerOverride(Double)} instead of faking gamepad
 * stick topics. While an override is set it wins over the stick and the auto-feed, until
 * {@link #clearPowerOverride()} hands control back.
 */
public class Elevator extends Node {

	/** Power applied while the Gate is open, the Shooter is at RPM, and the stick is centred. */
	public static double AUTO_POWER = 1.0;

	// Stick magnitudes inside this band count as centred, so a drifting stick is not an override.
	// Applied as a deadzone on the raw reading below.
	private static final double STICK_DEADBAND = 0.05;

	private final SafeDevice<DcMotorEx> elevator;
	private final SafeDevice.Scope<DcMotorEx> elevatorWrite;

	// Written by the stick, bumper, gate, Shooter, and auto callbacks, which Synapse dispatches on
	// separate threads. Each is a single field read back by apply(), so volatile is enough.
	private volatile double manualPower = 0.0;
	private volatile boolean bumperPressed;
	private volatile boolean triggerPressed;
	private volatile boolean gateOpen;
	private volatile boolean shooterAtRpm;

	// Set by auto (or any other caller) to drive the elevator directly. While non-null it wins over
	// the stick and the auto-feed; null hands control back to the gamepad.
	private volatile Double powerOverride;

	public Elevator(Orchestrator orchestrator, SafeDevice<DcMotorEx> elevator) {
		super(orchestrator);
		this.elevator = elevator;
		this.elevatorWrite = elevator.labels("write/elevator").slot("elevator/power");
		elevator.labels("write/elevator").run(m -> m.setDirection(DcMotorEx.Direction.REVERSE));
	}

	@SubscribedTo(topic = "g2/right_stick_y")
	public void onManualPower(Float stickY) {
		double raw = stickY != null ? -stickY.doubleValue() : 0.0;
		// Deadzone: ignore tiny deflections so stick drift doesn't creep the elevator.
		manualPower = Math.abs(raw) < STICK_DEADBAND ? 0.0 : raw;
		apply();
	}

//	@SubscribedTo(topic = "g2/left_bumper")
//	public void onBumper(Boolean pressed) {
//		bumperPressed = Boolean.TRUE.equals(pressed);
//		apply();
//	}

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

	// An explicit auto override wins; otherwise an off-centre stick, then a held bumper or left
	// trigger, then the auto-feed while the Gate is open and the Shooter is up to speed.
	private void apply() {
		double applied;
		if (powerOverride != null) {
			applied = powerOverride;
		} else if (Math.abs(manualPower) > STICK_DEADBAND) {
			applied = manualPower;
		} else if (bumperPressed || triggerPressed) {
			applied = AUTO_POWER;
		} else if (gateOpen && shooterAtRpm) {
			applied = AUTO_POWER;
		} else {
			applied = 0.0;
		}
		elevatorWrite.run(m -> m.setPower(applied));
		orchestrator.publish("elevator/power", applied);
	}

	/**
	 * Drive the elevator from auto: the power is latched and wins over the stick and the auto-feed
	 * until {@link #clearPowerOverride()} is called. Pass {@code null} to release the override.
	 */
	public void setPowerOverride(Double power) {
		powerOverride = power;
		apply();
	}

	/** Release a {@link #setPowerOverride(Double)} so the gamepad and auto-feed drive the elevator. */
	public void clearPowerOverride() {
		powerOverride = null;
		apply();
	}

	/** Whether auto currently owns the elevator. */
	public boolean isOverridden() {
		return powerOverride != null;
	}

	@Override
	public void close() {
		elevatorWrite.run(m -> m.setPower(0.0));
	}
}
