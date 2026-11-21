package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;

/**
 * Garden route: push out of the parking zone, shoot from the garden, grab a flower on the scoring half, and
 * park. The red and blue autos share it, and {@link MainAuto} lays the poses out on whichever half the
 * alliance starts on.
 */
public abstract class SoloAuto extends MainAuto {

	@Override
	protected Pose getStartingPose() {
		return poses.startSolo;
	}

	@Override
	public void setRoute() {
		addStep(AutoStates.leavePushAlliance);
		addStep(AutoStates.goToGardenShooting);
		addStep(AutoStates.pickFromFlowerScoring);
		addStep(AutoStates.parkScoring);
	}
}
