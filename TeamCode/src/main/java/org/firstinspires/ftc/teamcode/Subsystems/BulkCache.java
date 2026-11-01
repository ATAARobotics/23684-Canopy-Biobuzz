package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

/**
 * Manages REV Expansion Hub bulk caching so motor encoder reads across the robot share a single
 * I2C transaction per control cycle instead of one transaction per read.
 *
 * <p>Without bulk caching, every {@code DcMotorEx#getCurrentPosition()} or {@code #getVelocity()}
 * call is a discrete I2C transaction to the Expansion Hub. With bulk caching enabled, the Hub
 * returns all cached motor data in one transaction, which shortens control cycle times.
 *
 * <h3>Modes</h3>
 * <ul>
 *   <li>{@link Mode#OFF} - no cache and every read is a discrete transaction (slowest, freshest data).
 *   <li>{@link Mode#AUTO} - the Hub auto-updates its cache and one bulk-read per cycle unless you read
 *       the same encoder twice. Good compromise and no per-cycle user action required.
 *   <li>{@link Mode#MANUAL} - you must clear the cache once per cycle before any reads (one
 *       bulk-read per cycle, most efficient). This node clears it in its periodic method, so
 *       register it before Drive/Turret/Pedro so the clear runs first.
 * </ul>
 */
public class BulkCache extends Node {

	public enum Mode {
		OFF,
		AUTO,
		MANUAL
	}

	private final SafeHardwareMap safeMap;
	private final List<LynxModule> hubs;
	private final Mode mode;

	public BulkCache(Orchestrator orchestrator, SafeHardwareMap safeMap, Mode mode) {
		super(orchestrator);
		this.safeMap = safeMap;
		this.mode = mode;

		HardwareMap hw = safeMap.raw();
		hubs = hw.getAll(LynxModule.class);

		for (LynxModule hub : hubs) {
			hub.setBulkCachingMode(toLynxMode(mode));
		}

		orchestrator.publish("bulkcache/hubCount", hubs.size());
		orchestrator.publish("bulkcache/mode", mode.name());
	}

	/**
	 * Clears the bulk cache on every Expansion Hub. Required once per control cycle when using
	 * {@link Mode#MANUAL}, and must happen before any motor reads in that cycle. This method runs
	 * automatically from {@link #maintainCache()} when in MANUAL mode.
	 */
	public void clearBulkCache() {
		for (LynxModule hub : hubs) {
			hub.clearBulkCache();
		}
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void maintainCache() {
		if (mode == Mode.MANUAL) {
			clearBulkCache();
		}
	}

	private static LynxModule.BulkCachingMode toLynxMode(Mode mode) {
		switch (mode) {
			case OFF:
				return LynxModule.BulkCachingMode.OFF;
			case AUTO:
				return LynxModule.BulkCachingMode.AUTO;
			case MANUAL:
			default:
				return LynxModule.BulkCachingMode.MANUAL;
		}
	}
}
