package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedScoringSideAuto extends MainAuto {
	@Override
	protected Pose getStartingPose() {
		return new Pose(63, 133, Math.toRadians(270));
	}

	@Override
	protected boolean onRed() {
		return false;
	}

	@Override
	public void SetRoute() {
		addStep(15);
		addStep(AutoStates.leaveScoring);
		addStep(AutoStates.shootPreload);
//		addStep(AutoStates.pickFromFlowerScoring);
		addStep(AutoStates.parkScoring);
	}
}
