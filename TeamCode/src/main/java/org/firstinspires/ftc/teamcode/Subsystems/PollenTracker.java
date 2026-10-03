package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

/**
 * Polls the Limelight 3A's colour-blob pipeline and publishes the single CLOSEST blob.
 *
 * <p>"Closest" is the blob with the largest {@code getTargetArea()} — i.e. the one
 * taking up the most of the frame. The Limelight does the HSV thresholding and blob
 * finding on-board; this node only picks the winner and republishes it onto topics.
 *
 * <p>There is no target lock. Every poll re-selects the largest blob, so when the
 * current target is collected (or simply is no longer the largest) the next one is
 * picked up automatically with no extra state.
 *
 * <p>Polling happens on the hardware thread so the Limelight's USB/network traffic is
 * serialised with the rest of the robot's hardware access.
 */
public class PollenTracker extends Node {

	/** Matches the Limelight's own frame rate; the Drive loop runs at 50Hz too. */
	private static final int HZ = 50;

	/** Which of the Limelight's 10 pipelines holds the colour-blob pipeline. */
	private static final int PIPELINE_INDEX = 0;

	/** A result older than this is treated as "no target" so a dropped link stops the robot. */
	private static final long MAX_STALENESS_MS = 250;

	public static final String VALID = "pollen/valid";
	public static final String BEARING_DEG = "pollen/bearingDeg";
	public static final String AREA = "pollen/area";
	public static final String COUNT = "pollen/count";

	private final SafeDevice<Limelight3A> limelight;

	/** Device setup is deferred to the hardware thread so the calls keep their order. */
	private boolean started = false;

	public PollenTracker(Orchestrator orchestrator, SafeDevice<Limelight3A> limelight) {
		super(orchestrator);
		this.limelight = limelight;

		// Seed the topics so consumers never see "absent" before the first poll lands.
		orchestrator.publish(VALID, false);
		orchestrator.publish(BEARING_DEG, 0.0);
		orchestrator.publish(AREA, 0.0);
		orchestrator.publish(COUNT, 0);
	}

	@RunPeriodically(hz = HZ, hardware = true)
	public void poll() {
		Limelight3A ll = limelight.raw();

		if (!started) {
			ll.setPollRateHz(HZ);
			ll.pipelineSwitch(PIPELINE_INDEX);
			ll.start();
			started = true;
		}

		LLResult result = ll.getLatestResult();

		if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) {
			orchestrator.publish(VALID, false);
			return;
		}

		List<ColorResult> blobs = result.getColorResults();
		if (blobs == null || blobs.isEmpty()) {
			orchestrator.publish(VALID, false);
			return;
		}

		ColorResult closest = null;
		double largestArea = -1.0;
		for (ColorResult blob : blobs) {
			double area = blob.getTargetArea();
			if (area > largestArea) {
				largestArea = area;
				closest = blob;
			}
		}

		orchestrator.publish(VALID, true);
		orchestrator.publish(BEARING_DEG, closest.getTargetXDegrees());
		orchestrator.publish(AREA, largestArea);
		orchestrator.publish(COUNT, blobs.size());
	}
}
