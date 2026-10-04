package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import java.util.Locale;

/**
 * An immutable snapshot of one detected AprilTag cluster, with its resolved geometry already
 * computed. Produced by {@link AprilTagVision} each polling cycle and safe to hold onto afterwards,
 * so telemetry and any consumer of the vision data see the same numbers for a given frame.
 */
public final class ClusterTarget {

	/** Cluster name from the SDK tag library, e.g. {@code "RED SCORING"}. */
	public final String name;

	/** Abbreviated cluster name, e.g. {@code "RED"}. */
	public final String shortName;

	/** Fraction of the cluster's tags currently visible, 0-100. */
	public final int percentClusterFound;

	/** How many tags the cluster has in total, or 0 if the SDK did not say. */
	public final int tagsTotal;

	/** How many of those tags are visible right now, derived from {@link #percentClusterFound}. */
	public final int tagsVisible;

	// ---- raw camera-facing pose, exactly as the SDK reported it ----------------

	public final double rawX;
	public final double rawY;
	public final double rawZ;
	public final double rawYaw;
	public final double rawPitch;
	public final double rawRoll;
	public final double rawRange;
	public final double rawBearing;
	public final double rawElevation;

	// ---- scorable determination -----------------------------------------------

	/** True when the cluster's name starts with the letter for our alliance colour. */
	public final boolean allianceMatch;

	/** True when the cluster is right-side up, per the roll test. */
	public final boolean upright;

	/** True when this cluster is both ours and right-side up, so a scoring element can enter it. */
	public final boolean scorable;

	/** Absolute roll, in degrees. Exposed because the upright dead-band is tunable. */
	public final double absRoll;

	// ---- resolved geometry ---------------------------------------------------

	/** Camera-to-target vector in field axes, inches. */
	public final TargetingMath.ResolvedPoint resolved;

	// ---- frame freshness ------------------------------------------------------

	/** {@code frameAcquisitionNanoTime} reported by the SDK for this detection. */
	public final long acquisitionNanos;

	/** How long ago that frame was captured, in milliseconds, at the time of the snapshot. */
	public final double ageMs;

	/**
	 * True when this detection is older than the staleness budget, meaning the camera is not
	 * keeping up and the numbers should not be trusted for closed-loop control.
	 */
	public final boolean stale;

	ClusterTarget(
			String name,
			String shortName,
			int percentClusterFound,
			int tagsTotal,
			double rawX,
			double rawY,
			double rawZ,
			double rawYaw,
			double rawPitch,
			double rawRoll,
			double rawRange,
			double rawBearing,
			double rawElevation,
			boolean allianceMatch,
			boolean upright,
			double absRoll,
			TargetingMath.ResolvedPoint resolved,
			long acquisitionNanos,
			double ageMs,
			boolean stale) {
		this.name = name;
		this.shortName = shortName;
		this.percentClusterFound = percentClusterFound;
		this.tagsTotal = tagsTotal;
		this.tagsVisible = tagsTotal > 0 ? Math.round(percentClusterFound / 100.0f * tagsTotal) : 0;
		this.rawX = rawX;
		this.rawY = rawY;
		this.rawZ = rawZ;
		this.rawYaw = rawYaw;
		this.rawPitch = rawPitch;
		this.rawRoll = rawRoll;
		this.rawRange = rawRange;
		this.rawBearing = rawBearing;
		this.rawElevation = rawElevation;
		this.allianceMatch = allianceMatch;
		this.upright = upright;
		this.absRoll = absRoll;
		this.scorable = allianceMatch && upright;
		this.resolved = resolved;
		this.acquisitionNanos = acquisitionNanos;
		this.ageMs = ageMs;
		this.stale = stale;
	}

	/** How much to trust the geometry, worst factor deciding. */
	public enum Confidence {
		GOOD,
		FAIR,
		POOR;

		public String label() {
			return name().toLowerCase(Locale.US);
		}
	}

	// Thresholds for the confidence grade. A cluster pose is a multi-tag solve, so it sharpens as
	// more tags come into view and as the viewing angle squares up. Range matters because a distant
	// cluster covers fewer pixels, which costs angular precision.
	private static final int VISIBILITY_GOOD = 75;
	private static final int VISIBILITY_FAIR = 40;
	private static final double ANGLE_GOOD_DEG = 30.0;
	private static final double ANGLE_FAIR_DEG = 65.0;
	private static final double RANGE_GOOD_IN = 60.0;
	private static final double RANGE_FAIR_IN = 120.0;

	/**
	 * Grade the reading by its weakest aspect rather than averaging, because one bad factor spoils
	 * the whole pose: a clean angle solve from two visible tags still beats nothing, and a
	 * square-on single tag still misses the range by more than a distant four-tag fix.
	 */
	public Confidence confidence() {
		if (percentClusterFound < VISIBILITY_FAIR || absRoll > ANGLE_FAIR_DEG || resolved.range > RANGE_FAIR_IN) {
			return Confidence.POOR;
		}
		if (percentClusterFound < VISIBILITY_GOOD || absRoll > ANGLE_GOOD_DEG || resolved.range > RANGE_GOOD_IN) {
			return Confidence.FAIR;
		}
		return Confidence.GOOD;
	}

	/**
	 * Why the grade came out the way it did, in the terms that produced it: how many tags are
	 * visible and how far off-axis the cluster is being viewed.
	 */
	public String qualityNote() {
		StringBuilder note = new StringBuilder();
		if (tagsTotal > 1) {
			note.append(tagsVisible).append(" of ").append(tagsTotal).append(" tags");
		} else if (tagsTotal == 1) {
			note.append("single tag");
		} else {
			note.append(percentClusterFound).append("% of tags");
		}
		note.append(", ").append(Math.round(absRoll)).append(" deg off-axis");
		return note.toString();
	}

	/**
	 * The single most useful thing to change, or an empty string when there is nothing worth
	 * chasing. Ordered so the biggest lever wins: a stale frame makes every other number
	 * meaningless, and clearing an occlusion beats squaring up.
	 */
	public String advice() {
		if (stale) {
			return "camera is not keeping up, lower the resolution or raise decimation";
		}
		if (tagsTotal > 1 && tagsVisible < tagsTotal) {
			return "reposition to expose the hidden tags";
		}
		if (absRoll > ANGLE_FAIR_DEG) {
			return "badly off-axis, square up to the cell";
		}
		if (percentClusterFound < VISIBILITY_GOOD) {
			return "some tags occluded, try a different angle";
		}
		if (absRoll > ANGLE_GOOD_DEG) {
			return "angled, squaring up would sharpen this";
		}
		if (resolved.range > RANGE_GOOD_IN) {
			return String.format(Locale.US, "%.0f in out, closer is more accurate", resolved.range);
		}
		return "";
	}

	/** True when this target is safe to act on: ours, upright, and not a stale frame. */
	public boolean usable() {
		return scorable && !stale;
	}

	/**
	 * Short verdict on whether this cluster can be scored, and if not, why. Ordered so the most
	 * fundamental disqualifier comes first.
	 */
	public String status() {
		if (!allianceMatch) {
			return "other alliance";
		}
		if (!upright) {
			return "upside down";
		}
		if (stale) {
			return "stale frame";
		}
		return "usable";
	}

	/**
	 * Every measurement worth reading, spelled out rather than abbreviated. Two per line to keep it
	 * compact, since the Driver Station wraps anything long.
	 */
	public String actionLines() {
		return String.format(
						Locale.US,
						"  Range %7.1f in     Horizontal %7.1f in%n"
								+ "  Bearing %+7.1f deg  Elevation %+7.1f deg%n"
								+ "  Height above %+6.1f in   Off-axis %+6.1f deg",
						resolved.range,
						resolved.horizontalDistance,
						resolved.bearing,
						resolved.elevation,
						resolved.z,
						resolved.offAxis)
				+ String.format(Locale.US, "%n  Quality: %s", qualityNote())
				+ adviceLine();
	}

	/** The suggested fix, or nothing when there is nothing worth chasing. */
	private String adviceLine() {
		String advice = advice();
		return advice.isEmpty() ? "" : String.format(Locale.US, "%n  Try: %s", advice);
	}

	/** Header line: name, whether it can be scored, and how good the reading is. */
	public String headerLine() {
		return String.format(Locale.US, "%s  %s  [%s]", name, status(), confidence().label());
	}

	/** One line, for clusters that are not the current best. */
	public String compactLine() {
		return String.format(
				Locale.US, "%-14s %6.1f in  %-8s %s", name, resolved.range, confidence().label(), status());
	}

	/** Everything, for bench tuning. Not shown by default. */
	public String verboseBlock() {
		return String.format(
				Locale.US,
				"  raw xyz %6.1f %6.1f %6.1f  pry %5.1f %6.1f %6.1f  age %3.0fms%n"
						+ "  raw rbe %6.1f %6.1f %6.1f%n"
						+ "  mine    %6.1f %6.1f %6.1f",
				rawX,
				rawY,
				rawZ,
				rawPitch,
				rawRoll,
				rawYaw,
				ageMs,
				rawRange,
				rawBearing,
				rawElevation,
				resolved.range,
				resolved.bearing,
				resolved.elevation);
	}

	/** Single-line summary, handy for logging. */
	@Override
	public String toString() {
		return String.format(
				Locale.US,
				"ClusterTarget{%s found=%d%% scorable=%s %s}",
				name,
				percentClusterFound,
				scorable,
				resolved);
	}
}