package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;

/**
 * Audience-half route: shoot the preload, then park on the audience side. The red and blue autos share it,
 * and {@link MainAuto} lays the poses out on whichever half the alliance starts on.
 */
public abstract class AudienceSideAuto extends MainAuto {

	@Override
	protected Pose getStartingPose() {
		return poses.startAudience;
	}

	@Override
	public void setRoute() {
		addStep(1);
		addStep(AutoStates.shootPreload);
		addStep(AutoStates.parkAudience);
	}
}
