package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.PollenPursuit;
import org.firstinspires.ftc.teamcode.Subsystems.PollenTracker;

@TeleOp(name = "Pollen Pursuit Test", group = "Tests")
public class PollenPursuitTest extends SafeOpMode {

	private SafeDevice<DcMotorEx> frontLeft;
	private SafeDevice<DcMotorEx> frontRight;
	private SafeDevice<DcMotorEx> backLeft;
	private SafeDevice<DcMotorEx> backRight;
	private SafeDevice<Limelight3A> limelight;

	@Override
	protected void onSafeInit() {
		frontLeft = safeMap.device(DcMotorEx.class, "frontLeft");
		frontRight = safeMap.device(DcMotorEx.class, "frontRight");
		backLeft = safeMap.device(DcMotorEx.class, "backLeft");
		backRight = safeMap.device(DcMotorEx.class, "backRight");
		limelight = safeMap.device(Limelight3A.class, "limelight");

		GamepadAdaptor.attach(orchestrator, gamepad1, "g1");

		orchestrator.registerNode("drive", new Drive(orchestrator, frontLeft, frontRight, backLeft, backRight));
		orchestrator.registerNode("pollenTracker", new PollenTracker(orchestrator, limelight));
		orchestrator.registerNode("pollenPursuit", new PollenPursuit(orchestrator));
	}

	@Override
	protected void onSafeLoop() {
		telemetry.setMsTransmissionInterval(100);

		boolean valid =
				orchestrator.getLatestValue(PollenTracker.VALID, Boolean.class).orElse(false);

		telemetry.addData("Target", valid ? "locked" : "none");
		telemetry.addData(
				"Bearing in camera (deg)",
				"%.1f",
				orchestrator
						.getLatestValue(PollenTracker.BEARING_DEG, Double.class)
						.orElse(0.0));
		telemetry.addData(
				"Bearing in robot (deg)",
				"%.1f",
				orchestrator
						.getLatestValue(PollenPursuit.BEARING_ROBOT_DEG, Double.class)
						.orElse(0.0));
		telemetry.addData(
				"Area (pct)",
				"%.1f",
				orchestrator.getLatestValue(PollenTracker.AREA, Double.class).orElse(0.0));
		telemetry.addData(
				"Blobs seen",
				orchestrator.getLatestValue(PollenTracker.COUNT, Integer.class).orElse(0));
		telemetry.addLine();
		telemetry.addData(
				"Forward cmd",
				"%.2f",
				orchestrator.getLatestValue(Drive.CMD_FORWARD, Double.class).orElse(0.0));
		telemetry.addData(
				"Strafe cmd (right +)",
				"%.2f",
				orchestrator.getLatestValue(Drive.CMD_STRAFE, Double.class).orElse(0.0));
		telemetry.addData(
				"Turn cmd",
				"%.2f",
				orchestrator.getLatestValue(Drive.CMD_TURN, Double.class).orElse(0.0));
		telemetry.addLine();
		telemetry.addData("camera yaw offset (0 = ahead)", "%.0f deg", PollenPursuit.CAMERA_YAW_DEG);
		telemetry.addLine("hold A = pursue, B = stop");

		telemetry.update();
	}
}
