package org.firstinspires.ftc.teamcode.Subsystems.Vision;

/**
 * Turns a reported cluster pose into field-frame geometry we can drive and shoot from. Robot axes +X forward/+Y left/+Z
 * up; field +Z up with heading 0 at +X, CCW-positive.
 */
public final class TargetingMath {

	private TargetingMath() {}

	/** Geometry we drive and shoot from, plus where the POI sits. */
	public static final class ResolvedPoint {

		/** Field-frame horizontal offset from the camera lens to the POI, inches. */
		public final double x;
		public final double y;

		/** Height of the POI above the floor, inches. Absolute, not measured from the lens. */
		public final double z;

		/** Straight-line camera lens to POI in inches. Measured from the lens, not the launcher. */
		public final double range;

		/** Horizontal angle from the robot's facing, degrees: positive left, 0 dead ahead. */
		public final double bearing;

		/** Vertical angle from the lens to the POI, degrees: positive above horizontal, always in [-90, 90]. */
		public final double elevation;

		/** Horizontal distance camera to POI in inches, ignoring height. */
		public final double horizontalDistance;

		/** Compass angle in the field frame, degrees CCW from +X. This is not bearing. */
		public final double azimuth;

		/** Angle off the camera's optical axis, degrees. 0 means dead centre in the image. */
		public final double offAxis;

		ResolvedPoint(
				double x,
				double y,
				double z,
				double cameraHeightIn,
				double bearingDeg,
				double azimuthDeg,
				double offAxisDeg) {
			this.x = x;
			this.y = y;
			this.z = z;

			// z is absolute, but range and elevation come off the lens, so take the lens height back out.
			double aboveLens = z - cameraHeightIn;
			double distance = Math.sqrt(x * x + y * y + aboveLens * aboveLens);

			this.horizontalDistance = Math.hypot(x, y);
			this.range = distance;
			this.elevation = distance > 1e-9 ? Math.toDegrees(Math.asin(clamp(aboveLens / distance, -1.0, 1.0))) : 0.0;
			this.bearing = bearingDeg;
			this.azimuth = azimuthDeg;
			this.offAxis = offAxisDeg;
		}

		@Override
		public String toString() {
			return String.format(
					"xy=(%.1f, %.1f) z=%.1f range=%.1f bearing=%+.1f elev=%+.1f horiz=%.1f offAxis=%.1f",
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
	 * Turns a cluster pose into field-frame geometry relative to the robot's facing. Camera x/y/z in inches, exactly as
	 * the SDK reports them: camera frame, x right, y along the optical axis, z camera-up. Neither the mount tilt nor the
	 * mount yaw has been applied, so both are undone here.
	 */
	public static ResolvedPoint resolve(
			double cameraX,
			double cameraY,
			double cameraZ,
			double clusterYaw,
			double cameraYawDeg,
			double cameraElevationDeg,
			double cameraHeightIn,
			double robotHeadingDeg,
			PointOfInterest poi) {

		// SDK axes (right, along-axis, camera-up) -> robot axes (forward, left, up). The SDK reports the pose in the
		// camera's own frame with no mount rotation applied, so the mount tilt has to come out here. Without it the
		// whole tag height hides inside the forward component and up collapses to the camera-frame residual.
		double tiltRad = Math.toRadians(cameraElevationDeg);
		double cosTilt = Math.cos(tiltRad);
		double sinTilt = Math.sin(tiltRad);
		double forward = cameraY * cosTilt - cameraZ * sinTilt;
		double left = -cameraX;
		double up = cameraY * sinTilt + cameraZ * cosTilt;

		// Undo the mount yaw to land in the robot's frame.
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
		// Absolute height; the lens height can move without the cluster height moving with it.
		double fieldZ = cameraHeightIn + up;

		if (poi != null && poi.isEnabled()) {
			double offsetX = poi.effectiveOffsetX();
			double offsetY = poi.effectiveOffsetY();
			double offsetZ = poi.effectiveOffsetZ();

			if (poi.getFrame() == PointOfInterest.Frame.TARGET_FRAME_CLUSTER) {
				// Yaw only, no pitch or roll, so the offset stays put as the cluster turns.
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

		// Back into the robot frame. bearing is an aim error: positive left, negative right, 0 ahead.
		double bearingForward = fieldX * cosHeading + fieldY * sinHeading;
		double bearingLeft = -fieldX * sinHeading + fieldY * cosHeading;

		double bearingDeg = Math.hypot(bearingForward, bearingLeft) > 1e-9
				? Math.toDegrees(Math.atan2(bearingLeft, bearingForward))
				: 0.0;

		// Compass angle in the field frame.
		double azimuthDeg = Math.hypot(fieldX, fieldY) > 1e-9 ? Math.toDegrees(Math.atan2(fieldY, fieldX)) : 0.0;

		// Off-axis via the spherical law of cosines between the camera axis and the target direction. Height is
		// absolute by now, so pull the lens back out before measuring angles from it.
		double aboveLens = fieldZ - cameraHeightIn;
		double distance = Math.sqrt(fieldX * fieldX + fieldY * fieldY + aboveLens * aboveLens);
		double targetElevation = distance > 1e-9 ? Math.asin(clamp(aboveLens / distance, -1.0, 1.0)) : 0.0;
		double cameraElevation = Math.toRadians(cameraElevationDeg);
		// The camera axis points at field azimuth robotHeadingDeg + cameraYawDeg, not at cameraYawDeg alone.
		double azimuthSeparation =
				Math.toRadians(VisionCameraPose.angleDelta(azimuthDeg, robotHeadingDeg + cameraYawDeg));
		double cosOffAxis = Math.sin(targetElevation) * Math.sin(cameraElevation) + Math.cos(targetElevation) * Math.cos(cameraElevation) * Math.cos(azimuthSeparation);
		double offAxisDeg = Math.toDegrees(Math.acos(clamp(cosOffAxis, -1.0, 1.0)));

		return new ResolvedPoint(fieldX, fieldY, fieldZ, cameraHeightIn, bearingDeg, azimuthDeg, offAxisDeg);
	}

	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}
