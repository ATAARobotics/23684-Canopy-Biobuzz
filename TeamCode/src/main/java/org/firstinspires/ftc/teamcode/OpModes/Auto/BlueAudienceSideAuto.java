package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

// Audience-half auto for the blue alliance. Same route as RedAudienceSideAuto, flipped across the field.
@Autonomous
public class BlueAudienceSideAuto extends AudienceSideAuto {

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}
