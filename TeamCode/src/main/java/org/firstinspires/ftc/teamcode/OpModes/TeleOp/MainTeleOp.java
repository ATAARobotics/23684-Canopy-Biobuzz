package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Localization;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilTagVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.PointOfInterest;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionCameraPose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends SafeOpMode {

	private SafeDevice<DcMotorEx> frontLeft;
	private SafeDevice<DcMotorEx> frontRight;
	private SafeDevice<DcMotorEx> backLeft;
	private SafeDevice<DcMotorEx> backRight;
	private SafeDevice<DcMotorEx> intake;

	private Localization localization;
	private AprilTagVision vision;
	private Turret turret;

	@Override
	protected void onSafeInit() {
		frontLeft = safeMap.device(DcMotorEx.class, "frontLeft");
		frontRight = safeMap.device(DcMotorEx.class, "frontRight");
		backLeft = safeMap.device(DcMotorEx.class, "backLeft");
		backRight = safeMap.device(DcMotorEx.class, "backRight");
		intake = safeMap.device(DcMotorEx.class, "intake");

		GamepadAdaptor.attach(orchestrator, gamepad1, "g1");
		GamepadAdaptor.attach(orchestrator, gamepad2, "g2");

		// Localization
		localization = new Localization(orchestrator, safeMap, Constants.localizerConfig);
		localization.setStartPose(Constants.startPose);
		orchestrator.registerNode("localization", localization);

		// AprilTag cluster vision (camera "gobilda_camera", alliance from TurretConfig at init).
		VisionCameraPose cameraPose = new VisionCameraPose();
		PointOfInterest poi = new PointOfInterest("AIM"); // disabled by default
		vision = new AprilTagVision(orchestrator, safeMap.raw(), cameraPose, poi, TurretConfig.allianceLetter());
		orchestrator.registerNode("vision", vision);

		turret = new Turret(
				orchestrator,
				safeMap,
				safeMap.device(CRServo.class, "turret"),
				safeMap.device(DcMotorEx.class, "intake"),
				localization,
				vision,
				"g2");
		orchestrator.registerNode("turret", turret);

		orchestrator.registerNode("drive", new Drive(orchestrator, frontLeft, frontRight, backLeft, backRight));
		orchestrator.registerNode("intake", new Intake(orchestrator, intake));
	}

	@Override
	protected void onSafeLoop() {
		turret.publishTelemetry(telemetry);
		telemetry.addData(
				"Drive",
				"FL %+.2f  FR %+.2f  BL %+.2f  BR %+.2f",
				orchestrator.getLatestValue("drive/power/fl", Double.class).orElse(0.0),
				orchestrator.getLatestValue("drive/power/fr", Double.class).orElse(0.0),
				orchestrator.getLatestValue("drive/power/bl", Double.class).orElse(0.0),
				orchestrator.getLatestValue("drive/power/br", Double.class).orElse(0.0));
		telemetry.addData(
				"Intake",
				"%+.2f",
				orchestrator.getLatestValue("intake/power", Double.class).orElse(0.0));

		if (TurretConfig.verboseTelemetry) {
			turret.publishVerboseTelemetry(telemetry);
			vision.publishTelemetry(telemetry);
		}
		telemetry.update();
	}
}
