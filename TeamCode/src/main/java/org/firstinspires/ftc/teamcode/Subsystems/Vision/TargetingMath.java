package org.firstinspires.ftc.teamcode.Subsystems.Vision;

/**
 * Pure geometry for turning a reported AprilTag cluster pose into the numbers a robot actually
 * wants: how far to drive, how far to throw, and where to aim.
 *
 * <p>No hardware, no state, no allocation-heavy behaviour — everything here is static and cheap
 * enough to run per detection per frame.
 *
 * <h2>Frames and conventions</h2>
 *
 * The SDK hands us {@code ftcPose.x/y/z} using the FTC-facing axes
 * <b>{@code +X} right, {@code +Y} forward, {@code +Z} up</b>, measured from the camera lens. That
 * is the same labelling {@link VisionCameraPose} uses for the camera's translation.
 *
 * <ul>
 *   <li><b>Camera axes</b> — {@code +X} right, {@code +Y} down, {@code +Z} forward, origin at the
 *       lens. What the SDK actually measures in; never needed directly here.</li>
 *   <li><b>Robot axes</b> — {@code +X} forward, {@code +Y} left, {@code +Z} up.</li>
 *   <li><b>Field axes</b> — horizontal {@code +X}/{@code +Y} with {@code +Z} up, where robot
 *       heading {@code 0} means facing {@code +X} and heading increases counter-clockwise viewed
 *       from above. This matches the usual FTC odometry convention.</li>
 * </ul>
 */
public final class TargetingMath {

	private TargetingMath() {}

	/**
	 * A resolved measurement: the vector from the camera to the point of interest, expressed in the
	 * field frame, plus the derived shooting/driving quantities.
	 */
	public static final class ResolvedPoint {

		/** Field-frame offset from the camera to the POI, inches. */
		public final double x;
		public final double y;
		public final double z;

		/**
		 * Straight-line 3D distance from the camera to the POI, inches. This is the launch
		 * distance: how far the scoring element has to travel. Note it is measured from the
		 * camera lens, so add your launcher's offset to it if the shooter sits well ahead of or
		 * behind the camera.
		 */
		public final double range;

		/**
		 * Horizontal angle from the camera to the POI, in degrees, relative to the direction the
		 * robot is facing. Positive is to the left, negative to the right, {@code 0} dead ahead.
		 * This is the number to feed a heading controller.
		 */
		public final double bearing;

		/**
		 * Vertical angle to the POI, in degrees. Positive means the POI is above the camera, so a
		 * launcher must tilt up by this much. Always in {@code [-90, 90]}.
		 */
		public final double elevation;

		/**
		 * Horizontal distance from the camera to the POI in inches, ignoring height entirely.
		 * This is the "drive distance" a robot would travel to be directly under the cluster.
		 */
		public final double horizontalDistance;

		/**
		 * Compass angle of the POI in the field frame, degrees CCW from field +X, matching robot
		 * heading. Distinct from {@link #bearing}, which is measured from the robot's facing.
		 */
		public final double azimuth;

		/**
		 * Angle between the camera's optical axis and the direction to the POI, degrees. Zero means
		 * the target sits dead centre in the image.
		 *
		 * <p>The single most useful number for judging aim: a large value means the camera is
		 * pointed well away from the target, so either the mount pitch is wrong or the robot is not
		 * pointed at the cluster. It also shows how much headroom is left before the target leaves
		 * the frame.
		 */
		public final double offAxis;

		/**
		 * @param bearingDeg robot-relative horizontal angle, degrees, positive left
		 * @param azimuthDeg field-frame compass angle, degrees CCW from +X
		 * @param offAxisDeg angle from the camera's optical axis, degrees
		 */
		ResolvedPoint(double x, double y, double z, double bearingDeg, double azimuthDeg, double offAxisDeg) {
			this.x = x;
			this.y = y;
			this.z = z;

			double distance = Math.sqrt(x * x + y * y + z * z);

			this.horizontalDistance = Math.hypot(x, y);
			this.range = distance;
			this.elevation = distance > 1e-9 ? Math.toDegrees(Math.asin(clamp(z / distance, -1.0, 1.0))) : 0.0;
			this.bearing = bearingDeg;
			this.azimuth = azimuthDeg;
			this.offAxis = offAxisDeg;
		}

		@Override
		public String toString() {
			return String.format(
					"field=(%.1f, %.1f, %.1f) range=%.1f bearing=%+.1f elev=%+.1f horiz=%.1f offAxis=%.1f",
					x, y, z, range, bearing, elevation, horizontalDistance, offAxis);
		}
	}

	/**
	 * Resolve a cluster's reported pose into field-frame geometry relative to the robot's facing.
	 *
	 * <p>The chain is: SDK camera-facing axes -> robot axes -> field axes, applying the camera's
	 * mount yaw and the robot's heading, then adding the POI offset if one is enabled.
	 *
	 * @param cameraX      {@code ftcPose.x}, inches, right-positive
	 * @param cameraY      {@code ftcPose.y}, inches, forward-positive
	 * @param cameraZ      {@code ftcPose.z}, inches, up-positive
	 * @param clusterYaw   {@code ftcPose.yaw}, degrees, only used by the cluster-relative POI frame
	 * @param cameraYawDeg camera mount yaw, degrees ({@link VisionCameraPose#getYaw()})
	 * @param cameraElevationDeg camera tilt above horizontal, degrees
	 *        ({@link VisionCameraPose#getElevation()}). Only used to report the off-axis angle;
	 *        the rotation below deliberately ignores pitch, because {@code ftcPose} has already had
	 *        the camera's full orientation applied by the SDK.
	 * @param robotHeadingDeg robot heading on the field, degrees CCW from {@code +X}
	 * @param poi          point of interest, or {@code null} for the cluster's own pose
	 * @return resolved geometry, never {@code null}
	 */
	public static ResolvedPoint resolve(
			double cameraX,
			double cameraY,
			double cameraZ,
			double clusterYaw,
			double cameraYawDeg,
			double cameraElevationDeg,
			double robotHeadingDeg,
			PointOfInterest poi) {

		// SDK axes (right, forward, up) -> robot axes (forward, left, up).
		double forward = cameraY;
		double left = -cameraX;
		double up = cameraZ;

		// Undo the camera's mount yaw so the vector now sits in the robot's frame.
		double yawRad = Math.toRadians(cameraYawDeg);
		double cosYaw = Math.cos(yawRad);
		double sinYaw = Math.sin(yawRad);
		double robotX = forward * cosYaw - left * sinYaw;
		double robotY = forward * sinYaw + left * cosYaw;

		// Robot frame -> field frame.
		double headingRad = Math.toRadians(robotHeadingDeg);
		double cosHeading = Math.cos(headingRad);
		double sinHeading = Math.sin(headingRad);
		double fieldX = robotX * cosHeading - robotY * sinHeading;
		double fieldY = robotX * sinHeading + robotY * cosHeading;
		double fieldZ = up;

		if (poi != null && poi.isEnabled()) {
			double offsetX = poi.effectiveOffsetX();
			double offsetY = poi.effectiveOffsetY();
			double offsetZ = poi.effectiveOffsetZ();

			if (poi.getFrame() == PointOfInterest.Frame.TARGET_FRAME_CLUSTER) {
				// Rotate the offset so it stays fixed relative to the cluster. Only the yaw
				// component is honoured here: a cluster's opening faces horizontally, so rotation
				// about the up axis is the part that matters. Pitch and roll are deliberately not
				// applied, so do not read this as full orientation alignment.
				double totalYaw = VisionCameraPose.wrapDegrees(clusterYaw + cameraYawDeg + robotHeadingDeg);
				double totalYawRad = Math.toRadians(totalYaw);
				double cosTotal = Math.cos(totalYawRad);
				double sinTotal = Math.sin(totalYawRad);
				double rotatedX = offsetX * cosTotal - offsetY * sinTotal;
				double rotatedY = offsetX * sinTotal + offsetY * cosTotal;
				offsetX = rotatedX;
				offsetY = rotatedY;
			}

			fieldX += offsetX;
			fieldY += offsetY;
			fieldZ += offsetZ;
		}

		// Rotate the field-frame vector back into the robot's frame so bearing reads as an
		// aim error: positive left, negative right, zero straight ahead.
		double bearingForward = fieldX * cosHeading + fieldY * sinHeading;
		double bearingLeft = -fieldX * sinHeading + fieldY * cosHeading;

		double bearingDeg =
				Math.hypot(bearingForward, bearingLeft) > 1e-9
						? Math.toDegrees(Math.atan2(bearingLeft, bearingForward))
						: 0.0;

		// Compass angle of the target in the field frame.
		double azimuthDeg = Math.hypot(fieldX, fieldY) > 1e-9 ? Math.toDegrees(Math.atan2(fieldY, fieldX)) : 0.0;

		// Angle between the camera's optical axis and the target, via the spherical law of cosines.
		// Both directions are described by (azimuth, elevation): the camera's by its mount yaw and
		// tilt, the target's by where we just computed it to be.
		double distance = Math.sqrt(fieldX * fieldX + fieldY * fieldY + fieldZ * fieldZ);
		double targetElevation = distance > 1e-9 ? Math.asin(clamp(fieldZ / distance, -1.0, 1.0)) : 0.0;
		double cameraElevation = Math.toRadians(cameraElevationDeg);
		double azimuthSeparation = Math.toRadians(VisionCameraPose.angleDelta(azimuthDeg, cameraYawDeg));
		double cosOffAxis = Math.sin(targetElevation) * Math.sin(cameraElevation)
				+ Math.cos(targetElevation) * Math.cos(cameraElevation) * Math.cos(azimuthSeparation);
		double offAxisDeg = Math.toDegrees(Math.acos(clamp(cosOffAxis, -1.0, 1.0)));

		return new ResolvedPoint(fieldX, fieldY, fieldZ, bearingDeg, azimuthDeg, offAxisDeg);
	}

	/** Clamp {@code value} into {@code [min, max]}. */
	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}