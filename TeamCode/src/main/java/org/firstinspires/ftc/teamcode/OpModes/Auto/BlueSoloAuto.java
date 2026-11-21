package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utils.Team;

// Garden route for the blue alliance. Same route as RedSoloAuto, flipped across the field.
@Autonomous
public class BlueSoloAuto extends SoloAuto {

	@Override
	protected Team getTeam() {
		return Team.BLUE;
	}
}
