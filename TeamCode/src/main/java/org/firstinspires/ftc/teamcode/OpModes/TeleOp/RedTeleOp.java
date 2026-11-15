package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utils.Team;

// TeleOp for the red alliance. Shares all logic with MainTeleOp. Only the
// team and starting pose differ from BlueTeleOp.
@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleOp extends MainTeleOp {

	@Override
	protected Team getTeam() {
		return Team.RED;
	}

	@Override
	protected Pose getStartingPose() {
		// Red audience side, same as the previous shared start pose.
		return new Pose(63.0, 9.0, Math.PI / 2.0);
	}
}
