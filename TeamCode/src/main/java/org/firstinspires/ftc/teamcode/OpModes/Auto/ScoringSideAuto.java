package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;

/**
 * Scoring-half route: shoot the preload, then park on the scoring side. The red and blue autos share it,
 * and {@link MainAuto} lays the poses out on whichever half the alliance starts on.
 */
public abstract class ScoringSideAuto extends MainAuto {

	@Override
	protected Pose getStartingPose() {
		return poses.startScoring;
	}

	@Override
	public void setRoute() {
		addStep(15);
		addStep(AutoStates.shootPreload);
		addStep(AutoStates.parkScoring);
	}
}
