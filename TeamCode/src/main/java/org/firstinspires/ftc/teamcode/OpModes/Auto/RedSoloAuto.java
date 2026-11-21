package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

// Garden route for the red alliance.
@Autonomous
public class RedSoloAuto extends SoloAuto {

	@Override
	protected Team getTeam() {
		return Team.RED;
	}
}
