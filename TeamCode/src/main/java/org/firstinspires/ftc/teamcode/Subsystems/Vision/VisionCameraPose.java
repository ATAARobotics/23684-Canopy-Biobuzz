package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Camera mounting extrinsics relative to the robot center (inches, degrees).
 * SDK frame: +X right, +Y forward, +Z up; pitch 0 = straight up, -90 = horizontally forward; yaw + = left.
 */
public class VisionCameraPose {

	/** Default camera translation from the robot center, in inches. */
	public static final double DEFAULT_X = 0.0;
	public static final double DEFAULT_Y = 0.0;
	public static final double DEFAULT_Z = 0.0;

	/** Tilt above horizontal in degrees, where facing forward on the field is 0; this is the value you measure. */
	public static final double DEFAULT_ELEVATION_DEG = 55.4;

	/** SDK pitch: elevation - 90 (class note has the convention). Measure DEFAULT_ELEVATION_DEG, not this. */
	public static final double DEFAULT_PITCH = DEFAULT_ELEVATION_DEG - 90.0;

	public static final double DEFAULT_ROLL = 0.0;

	/** Turret resting yaw; mutable at runtime. */
	public static final double DEFAULT_YAW = 0.0;

	private double x = DEFAULT_X;
	private double y = DEFAULT_Y;
	private double z = DEFAULT_Z;
	private double pitch = DEFAULT_PITCH;
	private double roll = DEFAULT_ROLL;
	private double yaw = DEFAULT_YAW;

	public VisionCameraPose() {}

	public VisionCameraPose(VisionCameraPose other) {
		this.x = other.x;
		this.y = other.y;
		this.z = other.z;
		this.pitch = other.pitch;
		this.roll = other.roll;
		this.yaw = other.yaw;
	}

	/** Camera translation from the robot center, in inches. */
	public Position getPosition() {
		return new Position(DistanceUnit.INCH, x, y, z, 0L);
	}

	/** Camera orientation on the robot, in degrees; yaw is the turret axis. */
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

	/** SDK pitch: 0 = straight up, -90 = horizontal forward. Prefer setElevation for a measured tilt. */
	public void setPitch(double pitch) {
		this.pitch = pitch;
	}

	/** Tilt above horizontal: 0 = facing forward on the field, positive up, negative down. */
	public void setElevation(double elevationDeg) {
		this.pitch = elevationDeg - 90.0;
	}

	/** Current tilt above horizontal in degrees; inverse of setElevation. */
	public double getElevation() {
		return this.pitch + 90.0;
	}

	public void setRoll(double roll) {
		this.roll = roll;
	}

	/** Turret yaw in degrees: 0 = forward, +90 = left, -90 = right. */
	public void setYaw(double yaw) {
		this.yaw = yaw;
	}

	/** Summary in the intuitive frame: tilt above horizontal rather than the SDK's straight-up-is-0 pitch. */
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

	/** Add to turret yaw, wrapping to [-180, 180). */
	public void nudgeYaw(double deltaDegrees) {
		this.yaw = wrapDegrees(this.yaw + deltaDegrees);
	}

	public void resetToDefaults() {
		x = DEFAULT_X;
		y = DEFAULT_Y;
		z = DEFAULT_Z;
		pitch = DEFAULT_PITCH;
		roll = DEFAULT_ROLL;
		yaw = DEFAULT_YAW;
	}

	/** Loose equality to skip re-pushing an unchanged pose: angles within toleranceDegrees, xyz within 1e-6. */
	public boolean matches(VisionCameraPose other, double toleranceDegrees) {
		return other != null
				&& Math.abs(x - other.x) < 1e-6
				&& Math.abs(y - other.y) < 1e-6
				&& Math.abs(z - other.z) < 1e-6
				&& Math.abs(angleDelta(pitch, other.pitch)) < toleranceDegrees
				&& Math.abs(angleDelta(roll, other.roll)) < toleranceDegrees
				&& Math.abs(angleDelta(yaw, other.yaw)) < toleranceDegrees;
	}

	/** Wrap degrees into [-180, 180). */
	public static double wrapDegrees(double degrees) {
		double wrapped = degrees % 360.0;
		if (wrapped >= 180.0) {
			wrapped -= 360.0;
		} else if (wrapped < -180.0) {
			wrapped += 360.0;
		}
		return wrapped;
	}

	/** Shortest signed difference a - b, in degrees, wrapped to [-180, 180]. */
	public static double angleDelta(double a, double b) {
		return wrapDegrees(a - b);
	}

	@Override
	public String toString() {
		return String.format(
				"VisionCameraPose{xyz=(%.1f, %.1f, %.1f) in, ypr=(%.1f, %.1f, %.1f) deg}", x, y, z, yaw, pitch, roll);
	}
}
