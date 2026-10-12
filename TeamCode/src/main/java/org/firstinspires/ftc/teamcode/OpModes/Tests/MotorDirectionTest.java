package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Direction Test", group = "Tests")
public class MotorDirectionTest extends SafeOpMode {

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

		orch.subscribe("g1/x", Boolean.class, v -> frontLeft.run(m -> m.setPower(v ? 1.0 : 0.0)));
		orch.subscribe("g1/a", Boolean.class, v -> backLeft.run(m -> m.setPower(v ? 1.0 : 0.0)));
		orch.subscribe("g1/y", Boolean.class, v -> frontRight.run(m -> m.setPower(v ? 1.0 : 0.0)));
		orch.subscribe("g1/b", Boolean.class, v -> backRight.run(m -> m.setPower(v ? 1.0 : 0.0)));
		orch.subscribe("g1/left_bumper", Boolean.class, v -> intake.run(m -> m.setPower(v ? 1.0 : 0.0)));
	}

	@Override
	protected void onSafeLoop() {
		telemetry.addData("frontLeft", "X");
		telemetry.addData("backLeft", "A");
		telemetry.addData("frontRight", "Y");
		telemetry.addData("backRight", "B");
		telemetry.addData("intake", "Left Bumper");
		telemetry.update();
	}
}