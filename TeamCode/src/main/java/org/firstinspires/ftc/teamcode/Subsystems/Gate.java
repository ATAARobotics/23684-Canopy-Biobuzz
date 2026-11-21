package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A servo-actuated barrier below the Shooter that controls whether pollen can reach it. Opens once
 * the Shooter has been at target RPM for {@code AT_RPM_HOLD_NANOS} (200 ms) and closes otherwise.
 * Gamepad2's right bumper overrides it while held.
 */
public class Gate extends Node {

	private static final double CLOSED_POSITION = 0.0;
	private static final double OPEN_POSITION = 0.15;

	// Grace period the Shooter is held at RPM before the Gate opens, so the wheel is settled
	// before pollen starts moving. 200 ms is about 10 Shooter ticks.
	private static final long AT_RPM_HOLD_NANOS = 200_000_000L;

	// Guards overridden/lastAtRpm across trigger edges and Shooter ticks.
	private final Object stateLock = new Object();

	// Last commanded position, used for telemetry. open()/close() submit to the hardware thread
	// asynchronously, so this tracks intent instead of the servo's actual feedback.
	private volatile double position = CLOSED_POSITION;

	// Set while the right bumper is held. Shooter updates are ignored.
	private boolean overridden;

	// Last at-RPM state seen from the Shooter, so only a change moves the Gate.
	private boolean lastAtRpm;

	// When the Shooter first reported at-RPM, or 0 if it is not. While non-zero and less than
	// AT_RPM_HOLD_NANOS ago the Gate stays closed even though the Shooter is at speed.
	private long atRpmSinceNanos;

	private final SafeDevice<Servo> gate;

	public Gate(Orchestrator orchestrator, SafeDevice<Servo> gate) {
		super(orchestrator);
		this.gate = gate;
		orchestrator.getOrCreateTopic("gate/open", Boolean.class);
		// Start closed: hold pollen back until the Shooter is at speed. Queued under MatchGate.
		close();
	}

	@SubscribedTo(topic = "g2/right_bumper/rising")
	public void overrideOpen() {
		synchronized (stateLock) {
			overridden = true;
			atRpmSinceNanos = 0;
			open();
		}
	}

	@SubscribedTo(topic = "g2/right_bumper/falling")
	public void overrideClose() {
		synchronized (stateLock) {
			overridden = false;
			sync();
		}
	}

	// Republished every tick, so act when the at-RPM state flips or while it is still inside the
	// grace period. While overridden, just track it.
	@SubscribedTo(topic = "shooter/atRPM")
	public void onAtRpm(Boolean atRpm) {
		boolean atTarget = Boolean.TRUE.equals(atRpm);
		synchronized (stateLock) {
			boolean changed = atTarget != lastAtRpm;
			lastAtRpm = atTarget;
			if (overridden) {
				return;
			}
			if (!changed) {
				// Still at RPM: open the moment the grace period elapses.
				if (atTarget && atRpmSinceNanos != 0 && isHolding(nanos())) {
					sync();
				}
				return;
			}
			sync();
		}
	}

	/** Applies the current override/at-RPM state. Callers must hold {@code stateLock}. */
	private void sync() {
		if (overridden) {
			open();
			return;
		}
		if (!lastAtRpm) {
			atRpmSinceNanos = 0;
			close();
			return;
		}
		// At RPM: keep the Gate closed until the Shooter has been at speed for AT_RPM_HOLD_NANOS.
		long now = nanos();
		if (atRpmSinceNanos == 0) {
			atRpmSinceNanos = now;
		}
		if (isHolding(now)) {
			close();
		} else {
			open();
		}
	}

	/** Whether the grace period that starts when the Shooter reached RPM is still running. */
	private boolean isHolding(long now) {
		return now - atRpmSinceNanos < AT_RPM_HOLD_NANOS;
	}

	private static long nanos() {
		return System.nanoTime();
	}

	/** Close the Gate: servo position CLOSED_POSITION blocks pollen from reaching the Shooter. */
	public void close() {
		position = CLOSED_POSITION;
		gate.labels("write/gate").run(servo -> servo.setPosition(CLOSED_POSITION));
		orchestrator.publish("gate/open", false);
	}

	/** Open the Gate: servo position OPEN_POSITION clears the path so pollen can enter the Shooter. */
	public void open() {
		position = OPEN_POSITION;
		gate.labels("write/gate").run(servo -> servo.setPosition(OPEN_POSITION));
		orchestrator.publish("gate/open", true);
	}

	/** Whether the Gate was last commanded open. */
	public boolean isOpen() {
		return position == OPEN_POSITION;
	}

	/** Whether the right bumper currently overrides the Shooter. */
	public boolean isOverridden() {
		synchronized (stateLock) {
			return overridden;
		}
	}
}
