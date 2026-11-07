package org.firstinspires.ftc.teamcode.Config;

/**
 * One AprilTag cluster anchor on the field.
 * Immutable data carrier for cluster position and metadata.
 */
public final class Cluster {
	public final String name;
	public final String shortName;
	public final double x;
	public final double y;
	public final double z;

	public Cluster(String name, String shortName, double x, double y, double z) {
		this.name = name;
		this.shortName = shortName;
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * Horizontal (X/Y) distance from a field point to this anchor, in inches.
	 */
	public double horizontalDistanceFrom(double fromX, double fromY) {
		return Math.hypot(x - fromX, y - fromY);
	}
}