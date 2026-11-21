package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;

/** Holds every Synapse write until the driver confirms the match timer started, then replays what queued. */
public class MatchGate extends Node {

	/** Published true while writes are held, false once released. */
	public static final String HELD_TOPIC = "match/writesHeld";

	/** Share button: the driver confirms the match timer started. */
	public static final String RELEASE_BUTTON = "g1/share/rising";

	/** Options button: re-take the hold, for practice starts and re-runs. */
	public static final String REHOLD_BUTTON = "g1/options/rising";

	/**
	 * Override: publishing a value here forces the gate to that state, whatever the
	 * release/rehold buttons would say. Auto has no gamepad to press
	 * {@link #RELEASE_BUTTON}, so it publishes {@code false} on START to let its writes
	 * flow. The value is the desired held state, matching {@link #HELD_TOPIC}
	 * (true = hold writes, false = release them).
	 */
	public static final String OVERRIDE_TOPIC = "match/override";

	public MatchGate(Orchestrator orchestrator) {
		super(orchestrator);
		orchestrator.hardware().restrict("write").queue().sealChildren().apply();
		orchestrator.getOrCreateTopic(HELD_TOPIC, Boolean.class);
		orchestrator.publish(HELD_TOPIC, true);
		orchestrator.log("match gate: hardware writes held until " + RELEASE_BUTTON);
	}

	@SubscribedTo(topic = RELEASE_BUTTON)
	public void releaseWrites() {
		orchestrator.hardware().release("write");
		orchestrator.publish(HELD_TOPIC, false);
		orchestrator.log("match gate: released by driver, hardware writes flow");
	}

	@SubscribedTo(topic = REHOLD_BUTTON)
	public void holdWrites() {
		orchestrator.hardware().restrict("write").queue().sealChildren().apply();
		orchestrator.publish(HELD_TOPIC, true);
		orchestrator.log("match gate: re-held by driver");
	}

	@SubscribedTo(topic = OVERRIDE_TOPIC)
	public void override(Boolean held) {
		if (Boolean.TRUE.equals(held)) {
			orchestrator.hardware().restrict("write").queue().sealChildren().apply();
			orchestrator.publish(HELD_TOPIC, true);
			orchestrator.log("match gate: writes held by override");
		} else {
			orchestrator.hardware().release("write");
			orchestrator.publish(HELD_TOPIC, false);
			orchestrator.log("match gate: writes released by override");
		}
	}
}
