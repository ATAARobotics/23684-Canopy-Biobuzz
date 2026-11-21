package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A servo-actuated barrier below the Shooter that controls whether pollen can reach it. Opens
 * while the Shooter is at target RPM and closes otherwise. Gamepad2's right bumper overrides it
 * while held.
 */
public class Gate extends Node {

    private static final double CLOSED_POSITION = 0.0;
    private static final double OPEN_POSITION = 0.15;

    // Guards overridden/lastAtRpm across trigger edges and Shooter ticks.
    private final Object stateLock = new Object();

    // Last commanded position, used for telemetry. open()/close() submit to the hardware thread
    // asynchronously, so this tracks intent instead of the servo's actual feedback.
    private volatile double position = CLOSED_POSITION;

    // Set while the right bumper is held. Shooter updates are ignored.
    private boolean overridden;

    // Last at-RPM state seen from the Shooter, so only a change moves the Gate.
    private boolean lastAtRpm;

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

    // Republished every tick, so act only when it flips. While overridden, just track it.
    @SubscribedTo(topic = "shooter/atRPM")
    public void onAtRpm(Boolean atRpm) {
        boolean atTarget = Boolean.TRUE.equals(atRpm);
        synchronized (stateLock) {
            boolean changed = atTarget != lastAtRpm;
            lastAtRpm = atTarget;
            if (overridden || !changed) {
                return;
            }
            sync();
        }
    }

    /** Applies the current override/at-RPM state. Callers must hold {@code stateLock}. */
    private void sync() {
        if (overridden || lastAtRpm) {
            open();
        } else {
            close();
        }
    }

    /** Close the Gate: servo position CLOSED_POSITION blocks pollen from reaching the Shooter. */
    public void close() {
        position = CLOSED_POSITION;
        gate.run(servo -> servo.setPosition(CLOSED_POSITION));
        orchestrator.publish("gate/open", false);
    }

    /** Open the Gate: servo position OPEN_POSITION clears the path so pollen can enter the Shooter. */
    public void open() {
        position = OPEN_POSITION;
        gate.run(servo -> servo.setPosition(OPEN_POSITION));
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