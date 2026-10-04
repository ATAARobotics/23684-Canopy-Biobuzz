package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Camera mounting extrinsics for AprilTag work, expressed relative to the robot center.
 *
 * <p>SDK 12.0 defines the camera pose as a {@link Position} (translation) plus a
 * {@link YawPitchRollAngles} (orientation) pair. The axes used by {@code setCameraPose} are:
 *
 * <ul>
 *   <li>Translation: {@code +X} right, {@code +Y} forward, {@code +Z} up.</li>
 *   <li>Orientation: {@code pitch = 0} points the camera <i>straight up</i>, {@code pitch = -90}
 *       points it horizontally forward. {@code yaw = 0} is forward, {@code +90} is left, {@code -90}
 *       is right.</li>
 * </ul>
 *
 * <p>So a camera tilted {@code n} degrees below horizontal is {@code pitch = -n}, and one tilted
 * {@code n} degrees above horizontal is {@code pitch = -90 + n}.
 *
 * <p>Yaw is deliberately mutable so a rotating turret can re-aim the camera without a
 * redeploy; see {@link #setYaw(double)}. Every other value also has a setter, but changing them at
 * runtime is normally only useful while dialling in a new mounting.
 */
public class VisionCameraPose {

	/** Translation of the camera from the robot center, in inches. */
	public static final double DEFAULT_X = 0.0;
	public static final double DEFAULT_Y = 0.0;
	public static final double DEFAULT_Z = 0.0;

	/**
	 * How far the camera is tilted above horizontal, in degrees. This is the number that is
	 * physically meaningful when you measure the mounting, and it is measured from
	 * "facing forward on the field" being zero.
	 */
	public static final double DEFAULT_ELEVATION_DEG = 55.4;

	/**
	 * The same mounting expressed in the SDK's {@code setCameraPose} convention, where
	 * {@code pitch = 0} points the camera straight up and {@code pitch = -90} points it
	 * horizontally forward.
	 *
	 * <p>So {@code pitch = elevation - 90}. Do not set this by hand from the physical angle;
	 * change {@link #DEFAULT_ELEVATION_DEG} instead, which is the value you measure.
	 */
	public static final double DEFAULT_PITCH = DEFAULT_ELEVATION_DEG - 90.0;

	public static final double DEFAULT_ROLL = 0.0;

	/** Turret yaw. Mutable at runtime; this is the resting value. */
	public static final double DEFAULT_YAW = 0.0;

	private double x = DEFAULT_X;
	private double y = DEFAULT_Y;
	private double z = DEFAULT_Z;
	private double pitch = DEFAULT_PITCH;
	private double roll = DEFAULT_ROLL;
	private double yaw = DEFAULT_YAW;

	public VisionCameraPose() {}

	/** Copy constructor, useful for snapshotting a pose before experimenting with it. */
	public VisionCameraPose(VisionCameraPose other) {
		this.x = other.x;
		this.y = other.y;
		this.z = other.z;
		this.pitch = other.pitch;
		this.roll = other.roll;
		this.yaw = other.yaw;
	}

	/**
	 * Camera translation from the robot center. X is right-positive, Y forward, Z up, all in
	 * inches. Distances are in inches throughout this subsystem.
	 */
	public Position getPosition() {
		return new Position(DistanceUnit.INCH, x, y, z, 0L);
	}

	/** Camera orientation on the robot, in degrees. Yaw is the turret axis. */
	public YawPitchRollAngles getAngles() {
		return new YawPitchRollAngles(AngleUnit.DEGREES, (float) yaw, (float) pitch, (float) roll, 0L);
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getZ() {
		return z;
	}

	public double getPitch() {
		return pitch;
	}

	public double getRoll() {
		return roll;
	}

	public double getYaw() {
		return yaw;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public void setZ(double z) {
		this.z = z;
	}

	/**
	 * Set camera pitch in the SDK's convention, where {@code 0} is straight up and {@code -90} is
	 * horizontal forward. Prefer {@link #setElevation(double)} when you are working from a
	 * physical measurement.
	 */
	public void setPitch(double pitch) {
		this.pitch = pitch;
	}

	/**
	 * Set the tilt above horizontal in degrees, which is the intuitive direction: {@code 0} is
	 * facing forward on the field, positive looks up, negative looks down.
	 */
	public void setElevation(double elevationDeg) {
		this.pitch = elevationDeg - 90.0;
	}

	/** Current tilt above horizontal, in degrees. The inverse of {@link #setElevation(double)}. */
	public double getElevation() {
		return this.pitch + 90.0;
	}

	public void setRoll(double roll) {
		this.roll = roll;
	}

	/** Set turret yaw in degrees: {@code 0} forward, {@code +90} left, {@code -90} right. */
	public void setYaw(double yaw) {
		this.yaw = yaw;
	}

	/**
	 * Human-readable summary in the intuitive frame, where tilt is measured above horizontal
	 * rather than in the SDK's "straight up is 0" convention.
	 */
	public String describe() {
		return String.format(
				"yaw %+.1f, tilt %+.1f above horizontal, roll %+.1f, at (%.1f, %.1f, %.1f) in",
				yaw,
				getElevation(),
				roll,
				x,
				y,
				z);
	}

	/** Add to the current turret yaw, wrapping to {@code [-180, 180)}. */
	public void nudgeYaw(double deltaDegrees) {
		this.yaw = wrapDegrees(this.yaw + deltaDegrees);
	}

	/** Restore every value to the compile-time defaults. */
	public void resetToDefaults() {
		x = DEFAULT_X;
		y = DEFAULT_Y;
		z = DEFAULT_Z;
		pitch = DEFAULT_PITCH;
		roll = DEFAULT_ROLL;
		yaw = DEFAULT_YAW;
	}

	/**
	 * Whether this pose equals another within a small tolerance. Used to avoid re-pushing the
	 * camera pose to the processor on every single frame when nothing has actually changed.
	 */
	public boolean matches(VisionCameraPose other, double toleranceDegrees) {
		return other != null
				&& Math.abs(x - other.x) < 1e-6
				&& Math.abs(y - other.y) < 1e-6
				&& Math.abs(z - other.z) < 1e-6
				&& Math.abs(angleDelta(pitch, other.pitch)) < toleranceDegrees
				&& Math.abs(angleDelta(roll, other.roll)) < toleranceDegrees
				&& Math.abs(angleDelta(yaw, other.yaw)) < toleranceDegrees;
	}

	/** Wrap degrees into {@code [-180, 180)}. */
	public static double wrapDegrees(double degrees) {
		double wrapped = degrees % 360.0;
		if (wrapped >= 180.0) {
			wrapped -= 360.0;
		} else if (wrapped < -180.0) {
			wrapped += 360.0;
		}
		return wrapped;
	}

	/** Shortest signed difference {@code a - b}, in degrees, wrapped to {@code [-180, 180]}. */
	public static double angleDelta(double a, double b) {
		return wrapDegrees(a - b);
	}

	@Override
	public String toString() {
		return String.format(
				"VisionCameraPose{xyz=(%.1f, %.1f, %.1f) in, ypr=(%.1f, %.1f, %.1f) deg}",
				x, y, z, yaw, pitch, roll);
	}
}