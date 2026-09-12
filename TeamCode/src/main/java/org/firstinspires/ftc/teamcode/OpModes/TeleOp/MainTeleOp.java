package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends SafeOpMode {

	private SafeDevice<DcMotorEx> frontLeft;
	private SafeDevice<DcMotorEx> frontRight;
	private SafeDevice<DcMotorEx> backLeft;
	private SafeDevice<DcMotorEx> backRight;
	private SafeDevice<DcMotorEx> intake;

	@Override
	protected void onSafeInit() {
		frontLeft = safeMap.device(DcMotorEx.class, "frontLeft");
		frontRight = safeMap.device(DcMotorEx.class, "frontRight");
		backLeft = safeMap.device(DcMotorEx.class, "backLeft");
		backRight = safeMap.device(DcMotorEx.class, "backRight");
		intake = safeMap.device(DcMotorEx.class, "intake");

		GamepadAdaptor.attach(orch, gamepad1, "g1");
		GamepadAdaptor.attach(orch, gamepad2, "g2");

		orch.registerNode("drive", new Drive(orch, frontLeft, frontRight, backLeft, backRight));
		orch.registerNode("intake", new Intake(orch, intake));
	}

	@Override
	protected void onSafeLoop() {
		telemetry.addData("drive", "g1 left stick");
		telemetry.addData("intake", "g2 left stick Y");
		telemetry.update();
	}
}