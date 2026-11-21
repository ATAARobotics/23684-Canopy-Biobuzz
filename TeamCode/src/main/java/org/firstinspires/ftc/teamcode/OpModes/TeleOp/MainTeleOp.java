package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.OpModes.Auto.Poses;
import org.firstinspires.ftc.teamcode.Subsystems.BulkCache;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MatchGate;
import org.firstinspires.ftc.teamcode.Subsystems.Pedro;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilTagVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.PointOfInterest;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionCameraPose;
import org.firstinspires.ftc.teamcode.Utils.Team;

/**
 * Shared TeleOp base. RedTeleOp and BlueTeleOp supply the team and starting
 * pose. Everything else (drive, intake, shooter, turret, vision) is shared here.
 */
public abstract class MainTeleOp extends SafeOpMode {

	// Where the Hood is parked once the match gate releases hardware writes.
	private static final double HOOD_RELEASE_POSITION = 0.20;

	private SafeDevice<DcMotorEx> frontLeft;
	private SafeDevice<DcMotorEx> frontRight;
	private SafeDevice<DcMotorEx> backLeft;
	private SafeDevice<DcMotorEx> backRight;
	private SafeDevice<DcMotorEx> intake;
	private SafeDevice<DcMotorEx> elevator;
	private SafeDevice<DcMotorEx> shooter;
	private SafeDevice<DcMotorEx> shooterKicker;
	private SafeDevice<Servo> hoodServo;

	private BulkCache bulkCache;
	private Pedro pedro;
	private AprilTagVision vision;
	private Turret turret;
	private Elevator elevatorNode;
	private Gate gate;
	private Hood hood;
	private Intake intakeNode;
	private ShooterKicker shooterKickerNode;

	// Which alliance this TeleOp is for (BLUE or RED).
	protected abstract Team getTeam();

	// Starting pose for this alliance's side of the field.
	protected abstract Pose getStartingPose();

	@Override
	protected void onSafeInit() {
		// The gate holds hardware actions before any other subsystem attempts to run.
		orchestrator.registerNode("matchGate", new MatchGate(orchestrator));
		bulkCache = new BulkCache(orchestrator, safeMap, BulkCache.Mode.AUTO);
		orchestrator.registerNode("bulkcache", bulkCache);

		frontLeft = safeMap.device(DcMotorEx.class, "frontLeft");
		frontRight = safeMap.device(DcMotorEx.class, "frontRight");
		backLeft = safeMap.device(DcMotorEx.class, "backLeft");
		backRight = safeMap.device(DcMotorEx.class, "backRight");
		intake = safeMap.device(DcMotorEx.class, "intake");
		elevator = safeMap.device(DcMotorEx.class, "elevator");
		shooter = safeMap.device(DcMotorEx.class, "pollenShooter");
		shooterKicker = safeMap.device(DcMotorEx.class, "shooterKicker");
		hoodServo = safeMap.device(Servo.class, "hoodServo");

		GamepadAdaptor.attach(orchestrator, gamepad1, "g1");
		GamepadAdaptor.attach(orchestrator, gamepad2, "g2");

		pedro = new Pedro(orchestrator, safeMap, getStartingPose());
		if (Poses.finalPose.equals(new Pose(0, 0, 0))) {
			pedro.setStartPose(getStartingPose());
		} else {
			pedro.setStartPose(Poses.finalPose);
		}

		orchestrator.registerNode("pedro", pedro);

		VisionCameraPose cameraPose = new VisionCameraPose();
		PointOfInterest poi = new PointOfInterest("AIM");
		vision = new AprilTagVision(
				orchestrator, safeMap.raw(), cameraPose, poi, getTeam().allianceLetter());
		orchestrator.registerNode("vision", vision);

		turret = new Turret(
				orchestrator,
				safeMap,
				safeMap.device(CRServo.class, "turretServo1"),
				safeMap.device(CRServo.class, "turretServo2"),
				safeMap.device(CRServo.class, "turretServo3"),
				intake,
				pedro.follower(),
				"g2",
				getTeam().allianceLetter());
		orchestrator.registerNode("turret", turret);

		orchestrator.registerNode("drive", new Drive(orchestrator, frontLeft, frontRight, backLeft, backRight));
		intakeNode = new Intake(orchestrator, intake);
		orchestrator.registerNode("intake", intakeNode);

		orchestrator.registerNode("shooter", new Shooter(orchestrator, shooter, "g2"));
		shooterKickerNode = new ShooterKicker(orchestrator, shooterKicker);
		orchestrator.registerNode("shooterKicker", shooterKickerNode);
		hood = new Hood(orchestrator, hoodServo);
		orchestrator.registerNode("hood", hood);

		// The Hood sits at its default until the match gate releases hardware writes,
		// so nothing moves before the match timer starts.
		orchestrator.subscribe(MatchGate.HELD_TOPIC, Boolean.class, held -> {
			if (Boolean.FALSE.equals(held)) {
				hood.moveTo(HOOD_RELEASE_POSITION);
			}
		});

		elevatorNode = new Elevator(orchestrator, elevator);
		orchestrator.registerNode("elevator", elevatorNode);

		gate = new Gate(orchestrator, safeMap.device(Servo.class, "gate"));
		orchestrator.registerNode("gate", gate);

		// Release every override position so teleop starts with the drivers in control. Each
		// subsystem already defaults to released, so this is defensive, but it keeps init from
		// depending on whatever a previous caller (e.g. Auto) may have latched.
		intakeNode.clearPowerOverride();
		elevatorNode.clearPowerOverride();
		shooterKickerNode.clearPowerOverride();
		gate.clearPowerOverride();
		hood.clearPowerOverride();
		turret.setManualGoal(null);
	}

	@Override
	protected void onSafeLoop() {
		telemetry.addData(
				"Writes", orchestrator.hardware().isRestricted("write") ? "HELD, press share to release" : "GO");
		telemetry.addData(
				"FL",
				"%.2f",
				orchestrator.getLatestValue("drive/power/fl", Double.class).orElse(0.0));
		telemetry.addData(
				"FR",
				"%.2f",
				orchestrator.getLatestValue("drive/power/fr", Double.class).orElse(0.0));
		telemetry.addData(
				"BL",
				"%.2f",
				orchestrator.getLatestValue("drive/power/bl", Double.class).orElse(0.0));
		telemetry.addData(
				"BR",
				"%.2f",
				orchestrator.getLatestValue("drive/power/br", Double.class).orElse(0.0));
		telemetry.addData(
				"Intake",
				"%.2f",
				orchestrator.getLatestValue("intake/power", Double.class).orElse(0.0));
		telemetry.addData("Gate", gate.isOpen() ? "OPEN" : "CLOSED");
		telemetry.addData("Alliance", getTeam());
		telemetry.addData(
				"BulkCache",
				"%d hub(s), %s mode",
				orchestrator.getLatestValue("bulkcache/hubCount", Integer.class).orElse(0),
				orchestrator.getLatestValue("bulkcache/mode", String.class).orElse("?"));
		telemetry.addData(
				"Elevator",
				"%.2f",
				orchestrator.getLatestValue("elevator/power", Double.class).orElse(0.0));
		telemetry.addData(
				"Shooter RPM",
				"%.0f / %.0f",
				orchestrator.getLatestValue("shooter/currentRPM", Double.class).orElse(0.0),
				orchestrator.getLatestValue("shooter/targetRPM", Double.class).orElse(0.0));
		telemetry.addData(
				"Kicker",
				"%.2f",
				orchestrator.getLatestValue("shooterKicker/power", Double.class).orElse(0.0));
		telemetry.addData(
				"Hood",
				"%.2f",
				orchestrator.getLatestValue("hood/targetPosition", Double.class).orElse(0.0));
		vision.publishTelemetry(telemetry);

		turret.publishTelemetry(telemetry);
		if (TurretConfig.verboseTelemetry) {
			turret.publishVerboseTelemetry(telemetry);
		}
		telemetry.update();
	}
}
