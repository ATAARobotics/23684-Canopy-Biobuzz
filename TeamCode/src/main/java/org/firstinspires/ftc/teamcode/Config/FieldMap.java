package org.firstinspires.ftc.teamcode.Config;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

// BioBuzz AprilTag cluster anchors in Pedro field coordinates: inches from the field corner, Z in inches
// above the floor. Distances ignore Z, so only X/Y move the numbers. RED AUDIENCE is exact from the field
// model and the other three are its mirrors across the center line at 70.5 in. Every anchor is for upright
// tags. Upside-down variants are not covered yet.
public final class FieldMap {

	// Mirror axis: the field's center line, in Pedro inches.
	public static final double MIDLINE = 70.5;

	// RED AUDIENCE anchor, exact from the field model.
	public static final double RED_AUDIENCE_X = 57.35940;
	public static final double RED_AUDIENCE_Y = 49.66178;
	public static final double RED_AUDIENCE_Z = 57.21110;

	// One cluster's anchor on the field.
	public static final class Cluster {

		public final String name;
		public final String shortName;
		public final double x;
		public final double y;
		public final double z;

		Cluster(String name, String shortName, double x, double y, double z) {
			this.name = name;
			this.shortName = shortName;
			this.x = x;
			this.y = y;
			this.z = z;
		}

		// Horizontal (X/Y) distance from a field point to this anchor, in inches.
		public double horizontalDistanceFrom(double fromX, double fromY) {
			return Math.hypot(x - fromX, y - fromY);
		}
	}

	// All four clusters: RED AUDIENCE mirrored across the center lines gives the other three.
	public static final List<Cluster> CLUSTERS = Arrays.asList(
			new Cluster("RED AUDIENCE", "RA", RED_AUDIENCE_X, RED_AUDIENCE_Y, RED_AUDIENCE_Z),
			new Cluster("RED SCORING", "RS", RED_AUDIENCE_X, mirror(RED_AUDIENCE_Y), RED_AUDIENCE_Z),
			new Cluster("BLUE AUDIENCE", "BA", mirror(RED_AUDIENCE_X), RED_AUDIENCE_Y, RED_AUDIENCE_Z),
			new Cluster("BLUE SCORING", "BS", mirror(RED_AUDIENCE_X), mirror(RED_AUDIENCE_Y), RED_AUDIENCE_Z));

	// Mirrors one coordinate across the field center line.
	public static double mirror(double coordinate) {
		return 2.0 * MIDLINE - coordinate;
	}

	// Anchor for an alliance half. Blue plus scoring gives BLUE SCORING.
	public static Cluster forCell(boolean blue, boolean scoring) {
		String name = (blue ? "BLUE " : "RED ") + (scoring ? "SCORING" : "AUDIENCE");
		for (Cluster cluster : CLUSTERS) {
			if (cluster.name.equals(name)) {
				return cluster;
			}
		}
		throw new IllegalArgumentException("Unknown cell: " + name);
	}

	// "RA 12.3  RS 45.6  ..." horizontal distances from a field point, in inches, for telemetry.
	public static String formatDistances(double fromX, double fromY) {
		StringBuilder line = new StringBuilder();
		for (Cluster cluster : CLUSTERS) {
			if (line.length() > 0) {
				line.append("  ");
			}
			line.append(cluster.shortName);
			line.append(String.format(Locale.US, " %.1f", cluster.horizontalDistanceFrom(fromX, fromY)));
		}
		return line.toString();
	}

	private FieldMap() {}
}
