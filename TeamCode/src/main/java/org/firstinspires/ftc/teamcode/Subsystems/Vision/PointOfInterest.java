package org.firstinspires.ftc.teamcode.Subsystems.Vision;

/**
 * A point of interest expressed as a fixed field-frame offset from an AprilTag cluster's reported
 * pose.
 *
 * <p>This exists because a cluster's reported pose is itself already offset from the physical tag
 * surface: the SDK's cluster metadata carries a {@code fieldPosition} / {@code fieldOrientation}
 * pair, and in BIOBUZZ it points the pose at the centre of the cell's opening rather than at the
 * tags on the underside. So a POI here means "<i>this</i> many inches from wherever the cluster says
 * it is", not "this many inches from the printed tag".
 *
 * <p>The offset is defined in the <b>field</b> frame, so it does not rotate with the robot or with
 * the cluster. That makes it the right choice for "shoot the point 8 inches in front of the cell
 * opening" style geometry. For an offset that should follow the cluster's own orientation, use
 * TARGET_FRAME_CLUSTER instead.
 *
 * <p><b>Disabled by default.</b> Every offset is zero and {@link #isEnabled()} is {@code false},
 * which makes {@link #getOffsetX() getOffsetY() getOffsetZ()} collapse to a no-op and all
 * measurements report the cluster's own pose. Call {@link #setEnabled(boolean)} to switch it on.
 */
public class PointOfInterest {

	/** Frame the offset is expressed in. */
	public enum Frame {

		/**
		 * Offset is a fixed field-frame vector: {@code +X}/{@code +Y} are horizontal, {@code +Z} is
		 * up. It does not rotate with the robot or the cluster.
		 */
		FIELD,

		/**
		 * Offset rotates with the cluster's reported orientation, so it stays "8 inches in front of
		 * the cell" no matter which way the cluster faces.
		 */
		TARGET_FRAME_CLUSTER
	}

	/** Human-readable label, e.g. {@code "SCORE"}. Purely for telemetry. */
	private final String name;

	private final Frame frame;

	private double offsetX;
	private double offsetY;
	private double offsetZ;

	private boolean enabled;

	/**
	 * Create a disabled POI with a zero offset. Set the offsets, then call
	 * {@link #setEnabled(boolean) setEnabled(true)}.
	 *
	 * @param name label for telemetry
	 */
	public PointOfInterest(String name) {
		this(name, Frame.FIELD);
	}

	/**
	 * @param name label for telemetry
	 * @param frame frame the offset is expressed in
	 */
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

	/**
	 * Offsets are ignored while disabled, so {@link #effectiveOffsetX()} and friends return zero
	 * rather than the configured values.
	 */
	public boolean isEnabled() {
		return enabled;
	}

	/**
	 * Turn the POI on or off. When off, measurements fall back to the cluster's own reported pose,
	 * which makes this a safe thing to leave wired in permanently.
	 */
	public void setEnabled(boolean enabled) {
		this.enabled = enabled;
	}

	/** Configured offset along field X (or cluster-forward), in inches. Ignored when disabled. */
	public double effectiveOffsetX() {
		return enabled ? offsetX : 0.0;
	}

	/** Configured offset along field Y (or cluster-left), in inches. Ignored when disabled. */
	public double effectiveOffsetY() {
		return enabled ? offsetY : 0.0;
	}

	/** Configured offset along Z (up), in inches. Ignored when disabled. */
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

	/** Clear the offset and disable. */
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