package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAudienceSideAuto extends MainAuto {

	@Override
	protected Pose getStartingPose() {
		return new Pose(63, 9, Math.toRadians(90));
	}

	@Override
	protected boolean onRed() {
		return false;
	}

	public void setRoute() {
		addStep(1);
		addStep(AutoStates.shootPreload);
		addStep(AutoStates.parkAudience);
	}
}
