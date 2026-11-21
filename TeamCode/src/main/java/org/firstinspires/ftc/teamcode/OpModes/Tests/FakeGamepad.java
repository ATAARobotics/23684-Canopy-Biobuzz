package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Autonomous
public class FakeGamepad extends SafeOpMode {

	SafeDevice<DcMotorEx> intake;

	@Override
	protected void onSafeInit() {
		intake = safeMap.device(DcMotorEx.class, "intake");
		orchestrator.registerNode("intake", new Intake(orchestrator, intake));
	}

	@Override
	protected void onSafeLoop() {
		orchestrator.publish("g2/left_stick_y", 1.0f);
	}
}
