package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.HoldState;
import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Arrays;
import java.util.List;

/** Example of an auto built from states. */
@Autonomous(name = "Auto: State Machine Example", group = "Auto")
public class StateMachineExampleAuto extends StateMachineOpMode {

	@Override
	protected List<State> buildStates() {
		SafeDevice<DcMotorEx> intake = safeMap.device(DcMotorEx.class, "intake");

		return Arrays.asList(
				new WaitState("Settle", 0.75),
				HoldState.forSeconds("Intake on", 1.0, on -> intake.run(m -> m.setPower(on ? 1.0 : 0.0))),
				new WaitState("Pause", 0.5),
				HoldState.forSeconds("Intake again", 1.0, on -> intake.run(m -> m.setPower(on ? 1.0 : 0.0))),
				new WaitState("Finish", 0.25));
	}
}
