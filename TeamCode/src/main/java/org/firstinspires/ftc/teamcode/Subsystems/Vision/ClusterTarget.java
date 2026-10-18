package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import java.util.Locale;

/**
 * Immutable snapshot of one detected AprilTag cluster, with geometry already resolved for the frame
 * it came from, so a held reference keeps reporting that frame's numbers.
 */
public final class ClusterTarget {

	/** Cluster name from the SDK tag library, e.g. "RED SCORING". */
	public final String name;

	/** Abbreviated cluster name, e.g. "RED". */
	public final String shortName;

	/** Fraction of the cluster's tags currently visible, 0-100. */
	public final int percentClusterFound;

	/** How many tags the cluster has in total, or 0 if the SDK did not say. */
	public final int tagsTotal;

	/** How many tags are visible right now, derived from percentClusterFound. */
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

	/** True when this cluster is ours, so a scoring element can enter it. */
	public final boolean scorable;

	// ---- resolved geometry ---------------------------------------------------

	/** Camera-to-target vector in field axes, inches. */
	public final TargetingMath.ResolvedPoint resolved;

	// ---- frame freshness ------------------------------------------------------

	/** The SDK's frameAcquisitionNanoTime for this detection. */
	public final long acquisitionNanos;

	/** How long ago that frame was captured, in milliseconds, at the time of the snapshot. */
	public final double ageMs;

	/** True when this detection is older than the staleness budget, so control should not trust it. */
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
		this.scorable = allianceMatch;
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

	// Confidence grade thresholds: a multi-tag solve sharpens as more tags come into view, and a
	// distant cluster covers fewer pixels, costing angular precision.
	private static final int VISIBILITY_GOOD = 75;
	private static final int VISIBILITY_FAIR = 40;
	private static final double RANGE_GOOD_IN = 60.0;
	private static final double RANGE_FAIR_IN = 120.0;

	/** Grades by the weakest aspect, because one bad factor spoils the whole pose. */
	public Confidence confidence() {
		if (percentClusterFound < VISIBILITY_FAIR || resolved.range > RANGE_FAIR_IN) {
			return Confidence.POOR;
		}
		if (percentClusterFound < VISIBILITY_GOOD || resolved.range > RANGE_GOOD_IN) {
			return Confidence.FAIR;
		}
		return Confidence.GOOD;
	}

	/** Why the grade came out the way it did: how many tags are visible. */
	public String qualityNote() {
		StringBuilder note = new StringBuilder();
		if (tagsTotal > 1) {
			note.append(tagsVisible).append(" of ").append(tagsTotal).append(" tags");
		} else if (tagsTotal == 1) {
			note.append("single tag");
		} else {
			note.append(percentClusterFound).append("% of tags");
		}
		return note.toString();
	}

	/**
	 * The single most useful thing to change, or "" when nothing is worth chasing; biggest lever
	 * first, since a stale frame makes every other number meaningless.
	 */
	public String advice() {
		if (stale) {
			return "camera is not keeping up, lower the resolution or raise decimation";
		}
		if (tagsTotal > 1 && tagsVisible < tagsTotal) {
			return "reposition to expose the hidden tags";
		}
		if (percentClusterFound < VISIBILITY_GOOD) {
			return "some tags occluded, try a different angle";
		}
		if (resolved.range > RANGE_GOOD_IN) {
			return String.format(Locale.US, "%.0f in out, closer is more accurate", resolved.range);
		}
		return "";
	}

	/** True when this target is safe to act on: ours, and not a stale frame. */
	public boolean usable() {
		return scorable && !stale;
	}

	/** Short verdict on whether this cluster can be scored, and if not, why; most fundamental cause first. */
	public String status() {
		if (!allianceMatch) {
			return "other alliance";
		}
		if (stale) {
			return "stale frame";
		}
		return "usable";
	}

	/** Every measurement spelled out, two per line since the Driver Station wraps long lines. */
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
				resolved.offAxis) + String.format(Locale.US, "%n  Quality: %s", qualityNote()) + adviceLine();
	}

	// The suggested fix, or "" when there is nothing worth chasing.
	private String adviceLine() {
		String advice = advice();
		return advice.isEmpty() ? "" : String.format(Locale.US, "%n  Try: %s", advice);
	}

	/** Header line: name, whether it can be scored, and how good the reading is. */
	public String headerLine() {
		return String.format(
				Locale.US, "%s  %s  [%s]", name, status(), confidence().label());
	}

	/** One line, for clusters that are not the current best. */
	public String compactLine() {
		return String.format(
				Locale.US,
				"%-14s %6.1f in  %-8s %s",
				name,
				resolved.range,
				confidence().label(),
				status());
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
