package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utils.Team;

// TeleOp for the blue alliance. Shares all logic with MainTeleOp. Only the
// team and starting pose differ from RedTeleOp.
@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends MainTeleOp {

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}

	@Override
	protected Pose getStartingPose() {
		// Mirrored across the field midline from the red start pose.
		// Tune these coordinates to the actual blue-side starting position.
		return new Pose(78.0, 9.0, Math.PI / 2.0);
	}
}
