package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import java.util.Locale;

/**
 * One AprilTag cluster we detected.
 */
public final class ClusterTarget {

	/** Cluster name out of the SDK tag library, like "RED SCORING". */
	public final String name;

	/** Shortened cluster name, like "RED". */
	public final String shortName;

	/** How much of the cluster we can see, 0-100. */
	public final int percentClusterFound;

	/** Tag count for the whole cluster, or 0 if unknown. */
	public final int tagsTotal;

	/** Tags we can see right now. */
	public final int tagsVisible;

	// Raw axis and positional data from the SDK.

	public final double rawX;
	public final double rawY;
	public final double rawZ;
	public final double rawYaw;
	public final double rawPitch;
	public final double rawRoll;
	public final double rawRange;
	public final double rawBearing;
	public final double rawElevation;

	// Can we score in the cell?

	/** True if the cluster name starts with our alliance colour. */
	public final boolean allianceMatch;

	/** True if it's our cluster, so we can shoot scoring elements in it. */
	public final boolean scorable;

	// Where it actually is

	/** Target in field axes, inches: x/y are offsets from the lens, z is absolute height above the floor. */
	public final TargetingMath.ResolvedPoint resolved;

	// How old is this?

	/** frameAcquisitionNanoTime the SDK reported for this detection. */
	public final long acquisitionNanos;

	/** How old that frame was when we made this object, in ms. */
	public final double ageMs;

	/** True if the frame is too old to trust. */
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

	/** How much we trust the position. */
	public enum Confidence {
		GOOD,
		FAIR,
		POOR;

		public String label() {
			return name().toLowerCase(Locale.US);
		}
	}

	// Pose accuracy drops with fewer visible tags or greater distance.
	// Use these thresholds to decide how much to trust vision measurements.
	private static final int VISIBILITY_GOOD = 75;
	private static final int VISIBILITY_FAIR = 40;
	private static final double RANGE_GOOD_IN = 60.0;
	private static final double RANGE_FAIR_IN = 120.0;

	/** Grades off whichever factor is worst, since a chain is only as strong as its weakest link. */
	public Confidence confidence() {
		if (percentClusterFound < VISIBILITY_FAIR || resolved.range > RANGE_FAIR_IN) {
			return Confidence.POOR;
		}
		if (percentClusterFound < VISIBILITY_GOOD || resolved.range > RANGE_GOOD_IN) {
			return Confidence.FAIR;
		}
		return Confidence.GOOD;
	}

	/** Tells you how the grade was calculated. */
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
	 * Suggestions to improve detection quality, or "" if there isn't one. Stale comes first because nothing else
	 * matters until the camera catches up.
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

	/** True if it's ours and the frame is fresh enough to trust. */
	public boolean usable() {
		return scorable && !stale;
	}

	/** Whether we can score it, and if not, why. */
	public String status() {
		if (!allianceMatch) {
			return "other alliance";
		}
		if (stale) {
			return "stale frame";
		}
		return "usable";
	}

	/** Everything we measured for telemetry. */
	public String actionLines() {
		return String.format(
				Locale.US,
				"  Horizontal %7.1f in     Range %7.1f in%n"
						+ "  Bearing %+7.1f deg  Elevation %+7.1f deg%n"
						+ "  Height above floor %+6.1f in   Off-axis %+6.1f deg",
				resolved.horizontalDistance,
				resolved.range,
				resolved.bearing,
				resolved.elevation,
				resolved.z,
				resolved.offAxis) + String.format(Locale.US, "%n  Quality: %s", qualityNote()) + adviceLine();
	}

	// Prints advice if we have any.
	private String adviceLine() {
		String advice = advice();
		return advice.isEmpty() ? "" : String.format(Locale.US, "%n  Try: %s", advice);
	}

	/** Name, whether we can score it, and how good the detection is. */
	public String headerLine() {
		return String.format(
				Locale.US, "%s  %s  [%s]", name, status(), confidence().label());
	}

	/** One line for clusters that aren't the best one. */
	public String compactLine() {
		return String.format(
				Locale.US,
				"%-14s %6.1f in  %-8s %s",
				name,
				resolved.horizontalDistance,
				confidence().label(),
				status());
	}

	/** Highly verbose telemetry for debugging, not recommended during matches. */
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
