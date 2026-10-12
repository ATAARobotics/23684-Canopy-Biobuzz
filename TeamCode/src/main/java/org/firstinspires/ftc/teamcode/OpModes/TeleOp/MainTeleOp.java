package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Subsystems.BeamBreak;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shoot;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends SafeOpMode {

	private SafeDevice<DcMotorEx> frontLeft;
	private SafeDevice<DcMotorEx> frontRight;
	private SafeDevice<DcMotorEx> backLeft;
	private SafeDevice<DcMotorEx> backRight;
	private SafeDevice<DcMotorEx> intake;
	private SafeDevice<DcMotorEx> shooter;
	private SafeDevice<DigitalChannel> beambreak;

	@Override
	protected void onSafeInit() {
		frontLeft = safeMap.device(DcMotorEx.class, "frontLeft");
		frontRight = safeMap.device(DcMotorEx.class, "frontRight");
		backLeft = safeMap.device(DcMotorEx.class, "backLeft");
		backRight = safeMap.device(DcMotorEx.class, "backRight");
		intake = safeMap.device(DcMotorEx.class, "intake");
		shooter = safeMap.device(DcMotorEx.class, "shooter");
		beambreak = safeMap.device(DigitalChannel.class,"beambreak");

		GamepadAdaptor.attach(orchestrator, gamepad1, "g1");
		GamepadAdaptor.attach(orchestrator, gamepad2, "g2");

		orchestrator.registerNode("drive", new Drive(orchestrator, frontLeft, frontRight, backLeft, backRight));
		orchestrator.registerNode("intake", new Intake(orchestrator, intake));
		orchestrator.registerNode("shoot",new Shoot(orchestrator, shooter));
		orchestrator.registerNode("beambreak", new BeamBreak(orchestrator, beambreak));
	}

	@Override
	protected void onSafeLoop() {
		telemetry.addData("FL", "%.2f", orchestrator.getLatestValue("drive/power/fl", Double.class).orElse(0.0));
		telemetry.addData("FR", "%.2f", orchestrator.getLatestValue("drive/power/fr", Double.class).orElse(0.0));
		telemetry.addData("BL", "%.2f", orchestrator.getLatestValue("drive/power/bl", Double.class).orElse(0.0));
		telemetry.addData("BR", "%.2f", orchestrator.getLatestValue("drive/power/br", Double.class).orElse(0.0));
		telemetry.addData("Intake", "%.2f", orchestrator.getLatestValue("intake/power", Double.class).orElse(0.0));
		telemetry.addData("Shooter","%.2f", orchestrator.getLatestValue("shoot/power", Double.class).orElse(0.0));
		telemetry.update();
	}
}