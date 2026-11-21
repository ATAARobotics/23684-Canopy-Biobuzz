package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

// Audience-half auto for the red alliance.
@Autonomous
public class RedAudienceSideAuto extends AudienceSideAuto {

	@Override
	protected Team getTeam() {
		return Team.RED;
	}
}
