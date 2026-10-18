package org.firstinspires.ftc.teamcode.Subsystems.Vision;

/**
 * A fixed field-frame offset, in inches, from an AprilTag cluster's reported pose (already offset from the tag
 * surface). Disabled by default; a disabled POI ignores its offsets and reports the cluster's own pose.
 */
public class PointOfInterest {

	/** Frame the offset is expressed in. */
	public enum Frame {

		/** Fixed field-frame vector: +X/+Y horizontal, +Z up; does not rotate with the robot or cluster. */
		FIELD,

		/** Rotates with the cluster's reported orientation, so it stays in front of the cell regardless of facing. */
		TARGET_FRAME_CLUSTER
	}

	// Telemetry label, e.g. "SCORE".
	private final String name;

	private final Frame frame;

	private double offsetX;
	private double offsetY;
	private double offsetZ;

	private boolean enabled;

	/** Creates a disabled, zero-offset POI in Frame.FIELD. Set the offsets, then enable it. */
	public PointOfInterest(String name) {
		this(name, Frame.FIELD);
	}

	/** Creates a disabled, zero-offset POI in the given frame. */
	public PointOfInterest(String name, Frame frame) {
		this.name = name;
		this.frame = frame;
		this.enabled = false;
	}

	public String getName() {
		return name;
	}

	public Frame getFrame() {
		return frame;
	}

	/** Offsets are ignored while disabled, so the effective offsets read zero. */
	public boolean isEnabled() {
		return enabled;
	}

	/** Turn the POI on or off; when off, measurements fall back to the cluster's own reported pose. */
	public void setEnabled(boolean enabled) {
		this.enabled = enabled;
	}

	/** Configured offset along X (field X, or cluster-forward), in inches. */
	public double effectiveOffsetX() {
		return enabled ? offsetX : 0.0;
	}

	/** Configured offset along Y (field Y, or cluster-left), in inches. */
	public double effectiveOffsetY() {
		return enabled ? offsetY : 0.0;
	}

	/** Configured offset along Z (up), in inches. */
	public double effectiveOffsetZ() {
		return enabled ? offsetZ : 0.0;
	}

	public double getOffsetX() {
		return offsetX;
	}

	public double getOffsetY() {
		return offsetY;
	}

	public double getOffsetZ() {
		return offsetZ;
	}

	public void setOffsetX(double offsetX) {
		this.offsetX = offsetX;
	}

	public void setOffsetY(double offsetY) {
		this.offsetY = offsetY;
	}

	public void setOffsetZ(double offsetZ) {
		this.offsetZ = offsetZ;
	}

	/** Set all three offsets at once, in inches. */
	public void setOffset(double x, double y, double z) {
		this.offsetX = x;
		this.offsetY = y;
		this.offsetZ = z;
	}

	/** Zeroes all three offsets and disables the POI. */
	public void reset() {
		offsetX = 0.0;
		offsetY = 0.0;
		offsetZ = 0.0;
		enabled = false;
	}

	@Override
	public String toString() {
		if (!enabled) {
			return String.format("POI[%s disabled]", name);
		}
		return String.format("POI[%s %s (%+.1f, %+.1f, %+.1f) in]", name, frame, offsetX, offsetY, offsetZ);
	}
}
