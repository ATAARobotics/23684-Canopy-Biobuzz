package org.firstinspires.ftc.teamcode.Subsystems.Vision;

/**
 * Pure geometry from a reported cluster pose to drive/throw/aim numbers.
 * Robot axes +X forward/+Y left/+Z up; field +Z up with heading 0 at +X, CCW-positive.
 */
public final class TargetingMath {

	private TargetingMath() {}

	/**
	 * Camera-to-POI offset in the field frame, plus derived shooting/driving quantities.
	 */
	public static final class ResolvedPoint {

		/** Field-frame offset from the camera to the POI, inches. */
		public final double x;
		public final double y;
		public final double z;

		/** Straight-line distance camera lens -> POI, inches; measured from the lens, not the launcher. */
		public final double range;

		/** Horizontal angle from the robot's facing to the POI, degrees: positive left, 0 dead ahead. */
		public final double bearing;

		/** Vertical angle to the POI, degrees: positive above the camera, always in [-90, 90]. */
		public final double elevation;

		/** Horizontal distance from the camera to the POI, inches, ignoring height. */
		public final double horizontalDistance;

		/** Compass angle of the POI in the field frame, degrees CCW from +X; distinct from bearing. */
		public final double azimuth;

		/** Angle between the camera's optical axis and the POI, degrees; 0 means dead centre in the image. */
		public final double offAxis;

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
					x,
					y,
					z,
					range,
					bearing,
					elevation,
					horizontalDistance,
					offAxis);
		}
	}

	/**
	 * Resolve a cluster pose into field-frame geometry relative to the robot's facing; camera x/y/z in inches.
	 * Mount pitch is deliberately not rotated (the SDK applied the camera's orientation); it only feeds offAxis.
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
				// Rotate with the cluster's yaw so the offset stays fixed to the cluster's opening;
				// pitch and roll are deliberately not applied, so this is not full orientation alignment.
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

		// Back into the robot frame; bearing is an aim error: positive left, negative right, zero ahead.
		double bearingForward = fieldX * cosHeading + fieldY * sinHeading;
		double bearingLeft = -fieldX * sinHeading + fieldY * cosHeading;

		double bearingDeg = Math.hypot(bearingForward, bearingLeft) > 1e-9
				? Math.toDegrees(Math.atan2(bearingLeft, bearingForward))
				: 0.0;

		// Compass angle of the target in the field frame.
		double azimuthDeg = Math.hypot(fieldX, fieldY) > 1e-9 ? Math.toDegrees(Math.atan2(fieldY, fieldX)) : 0.0;

		// Off-axis angle via the spherical law of cosines between the camera axis (mount yaw, tilt)
		// and the target direction (its azimuth and elevation).
		double distance = Math.sqrt(fieldX * fieldX + fieldY * fieldY + fieldZ * fieldZ);
		double targetElevation = distance > 1e-9 ? Math.asin(clamp(fieldZ / distance, -1.0, 1.0)) : 0.0;
		double cameraElevation = Math.toRadians(cameraElevationDeg);
		double azimuthSeparation = Math.toRadians(VisionCameraPose.angleDelta(azimuthDeg, cameraYawDeg));
		double cosOffAxis = Math.sin(targetElevation) * Math.sin(cameraElevation) + Math.cos(targetElevation) * Math.cos(cameraElevation) * Math.cos(azimuthSeparation);
		double offAxisDeg = Math.toDegrees(Math.acos(clamp(cosOffAxis, -1.0, 1.0)));

		return new ResolvedPoint(fieldX, fieldY, fieldZ, bearingDeg, azimuthDeg, offAxisDeg);
	}

	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}
