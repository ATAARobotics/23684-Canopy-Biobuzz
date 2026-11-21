package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAudienceSide extends MainAuto {

	@Override
	protected Pose getStartingPose() {
		return new Pose(55, 9, Math.toRadians(90));
	}

	@Override
	protected boolean onRed() {
		return false;
	}

	public void SetRoute() {
		addStep(AutoStates.leaveAudience);
		addStep(1);
		// addStep(AutoStates.shootPreload);
		addStep(AutoStates.pickFromFlowerAudience);
		addStep(AutoStates.goToGarden);
		addStep(AutoStates.parkAudience);
	}
}
