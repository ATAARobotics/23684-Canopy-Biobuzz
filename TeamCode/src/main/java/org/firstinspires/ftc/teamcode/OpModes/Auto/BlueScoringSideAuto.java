package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

// Scoring-half auto for the blue alliance. Same route as RedScoringSideAuto, flipped across the field.
@Autonomous
public class BlueScoringSideAuto extends ScoringSideAuto {

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}
