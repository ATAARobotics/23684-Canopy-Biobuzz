package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

// Audience-half auto for the blue alliance. Same route as RedAudienceSideAuto, flipped across the field,
// and therefore runs the scoring side auto.
@Autonomous
public class BlueAudienceSideAuto extends ScoringSideAuto {

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}
