package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.AbstractState;
import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.Subsystems.BulkCache;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class MainAuto extends StateMachineOpMode {

	public enum AutoStates {
		shootPreload,
		leaveScoring,
		leaveAudience,
		leavePushAlliance,
		goToGarden,
		goToGardenShooting,
		pickFromFlowerAudience,
		pickFromFlowerScoring,
		parkScoring,
		parkAudience,
		shoot
	}

	List<State> autoStates;
	Follower follower;

	Pose lastPath;

	Poses poses;

	private SafeDevice<DcMotorEx> intake;

	private BulkCache bulkCache;
	private Shooter shooter;
	private ShooterKicker shooterKicker;
	private Hood hood;
	private Turret turret;

	private Gate gate;
	private Elevator elevator;
	private Intake intakeNode;

	public void SetRoute() {}

	public void addStep(AutoStates autostates) {
		autoStates.add(GetState(autostates));
	}

	public void addStep(int wait) {
		autoStates.add(new WaitState(wait));
	}

	protected abstract Pose getStartingPose();

	protected abstract boolean onRed(); // will become the alliance swap thing

	private State GetState(AutoStates autoStates) {

		switch (autoStates) {
			case shootPreload:
				return ShootingSub();
			case leaveScoring:
				return new LeaveScoringState();
			case leaveAudience:
				return new LeaveAudienceState();
			case leavePushAlliance:
				return new LeavePushAlliance();
			case goToGarden:
				return GardenSub(false);
			case goToGardenShooting:
				return GardenSub(true);
			case pickFromFlowerScoring:
				return FlowerScoringSub();
			case pickFromFlowerAudience:
				return FlowerAudienceSub();
			case parkScoring:
				return new ParkScoringState();
			case parkAudience:
				return new ParkAudienceState();
			case shoot:
				return ShootingSub();
		}
		return null;
	}

	private final class ShootPreloadState extends ShootState {
		ShootPreloadState() {
			super("ShootPreload");
		}
	}

	/** Spins the shooter up to RPM, waits for it to settle, then drops the target. The kicker follows. */
	private class ShootState extends AbstractState {

		private final String stateName;
		private boolean settled;
		private long settledNanos;

		ShootState(String stateName) {
			this.stateName = stateName;
		}

		@Override
		public void init() {
			settled = false;
			settledNanos = 0L;
			startTimer(SHOOT_TIMEOUT_SECONDS);
			// Clear the path before the Shooter spins up: the override holds the Gate open
			// regardless of RPM, so pollen can reach the wheel the moment it is up to speed.
			gate.setPowerOverride(true);
			// Same for the kicker: hold it at a fixed power for the whole shoot instead of
			// letting the Shooter's target RPM decide.
			shooterKicker.setPowerOverride(0.6);
			orchestrator.publish("shooter/targetRPM", Shooter.RPM_TARGET);
			setEndCondition(
					() -> (settled && elapsedSince(settledNanos) >= SHOOT_SETTLE_SECONDS) || timer().hasElapsed());
		}

		@Override
		public void loop() {
			Boolean atRpm =
					orchestrator.getLatestValue("shooter/atRPM", Boolean.class).orElse(false);
			if (!settled && Boolean.TRUE.equals(atRpm)) {
				settled = true;
				settledNanos = System.nanoTime();
			}
			telemetry.addData(
					stateName,
					"target %.0f   current %.0f   atRPM %s   settled %s",
					Shooter.RPM_TARGET,
					orchestrator
							.getLatestValue("shooter/currentRPM", Double.class)
							.orElse(0.0),
					atRpm,
					settled);
		}

		@Override
		public String name() {
			return stateName;
		}
	}

	// How long the shooter holds RPM before moving on, and the cap before a stalled shooter is skipped.
	private static final double SHOOT_SETTLE_SECONDS = 0.25;
	private static final double SHOOT_TIMEOUT_SECONDS = 4.0;

	private static long elapsedSince(long startNanos) {
		return (System.nanoTime() - startNanos) / 1_000_000_000L;
	}

	private class ShooterStop extends AbstractState {
		float speed = 0f;

		@Override
		public void init() {
			// Shooting is done: close the Gate and let it follow the Shooter again.
			gate.setPowerOverride(false);
			// Hand the kicker back to the Shooter's RPM as well.
			shooterKicker.clearPowerOverride();
			setEndCondition(() -> Math.abs(speed) > 0);
		}

		@Override
		public void loop() {
			orchestrator.publish("shooter/targetRPM", 0.0f);
			speed = orchestrator
					.getLatestValue("shooter/targetRPM", Float.class)
					.orElse(0.0f);
		}
	}

	private Submachine ShootingSub() {
		return new Submachine(
				"Shooter",
				Arrays.asList(
						new ShootState("shooting Sub"),
						new RunTranferState(),
						new WaitState(0.2),
						new StopTranferState(),
						new ShooterStop()));
	}

	private final class RunTranferState extends AbstractState {
		float speed = 0f;

		@Override
		public void init() {
			// Drive the elevator directly instead of faking g2/right_stick_y: the override wins over
			// the auto-feed until StopTranferState clears it.
			elevator.setPowerOverride(1.0);
			setEndCondition(() -> Math.abs(speed) > 0);
		}

		@Override
		public void loop() {
			speed = orchestrator
					.getLatestValue("elevator/power", Double.class)
					.orElse(0.0)
					.floatValue();
		}
	}

	private final class StopTranferState extends AbstractState {
		float speed = 0f;

		@Override
		public void init() {
			// Hand the elevator back to the auto-feed.
			elevator.clearPowerOverride();
			setEndCondition(() -> Math.abs(speed) == 0);
		}

		@Override
		public void loop() {
			speed = orchestrator
					.getLatestValue("elevator/power", Double.class)
					.orElse(0.0)
					.floatValue();
		}
	}

	private final class RunIntake extends AbstractState {
		float speed = 0f;

		@Override
		public void init() {
			// Drive the intake directly instead of faking g2/left_stick_y: the override wins over
			// the stick and bumper/trigger until StopIntake clears it.
			intakeNode.setPowerOverride(1.0);
			setEndCondition(() -> Math.abs(speed) > 0);
		}

		@Override
		public void loop() {
			speed = orchestrator
					.getLatestValue("intake/power", Double.class)
					.orElse(0.0)
					.floatValue();
		}
	}

	private final class StopIntake extends AbstractState {
		float speed = 0f;

		@Override
		public void init() {
			// Hand the intake back to the gamepad.
			intakeNode.clearPowerOverride();
			setEndCondition(() -> Math.abs(speed) == 0);
		}

		@Override
		public void loop() {
			speed = orchestrator
					.getLatestValue("intake/power", Double.class)
					.orElse(0.0)
					.floatValue();
		}
	}

	private final class LeaveScoringState extends AbstractState {
		Path leavepath;

		@Override
		public void init() {
			leavepath = Paths.line(lastPath, poses.redScoringLeave).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavepath);
		}

		@Override
		public void loop() {
			follower.update();
			telemetry.addData(
					"isdone?",
					Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			telemetry.addData("Velocity", follower.tangentialVelocity());
			telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
			telemetry.addData("distance to end", follower.distanceToEndpoint());
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redAudienceLeave;
		}
	}

	private final class LeaveAudienceState extends AbstractState {
		Path leavepath;

		@Override
		public void init() {
			leavepath = Paths.line(lastPath, poses.redAudienceLeave).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavepath);
		}

		@Override
		public void loop() {
			follower.update();
			telemetry.addData(
					"isdone?",
					Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			telemetry.addData("Velocity", follower.tangentialVelocity());
			telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
			telemetry.addData("distance to end", follower.distanceToEndpoint());
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redAudienceLeave;
		}
	}

	private final class LeavePushAlliance extends AbstractState {
		Path leavepath;

		@Override
		public void init() {
			leavepath = Paths.line(lastPath, poses.redParkingZoneAlliancePush).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavepath);
		}

		@Override
		public void loop() {
			follower.update();
			telemetry.addData(
					"isdone?",
					Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			telemetry.addData("Velocity", follower.tangentialVelocity());
			telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
			telemetry.addData("distance to end", follower.distanceToEndpoint());
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redParkingZoneAlliancePush;
		}
	}

	private Submachine FlowerScoringSub() {

		return new Submachine(
				"Flower", Arrays.asList(new ToScoringFlowerState(), new WaitState(1), new LeaveScoringFlowerState()));
	}

	private final class ToScoringFlowerState extends AbstractState {

		Path goToFlower;
		Path leaveFlower;

		public void init() {
			if (lastPath.y() > 72) goToFlower = Paths.line(lastPath, poses.redScoringFlowerIntake).constant(Math.toRadians(90));
			else goToFlower = Paths.curve(lastPath, poses.firstFlowerControl1, poses.redScoringFlowerIntake)
					.linear(lastPath, poses.redScoringFlowerIntake);
			setEndCondition(() -> !follower.following());
			follower.follow(goToFlower);
		}

		@Override
		public void loop() {
			follower.update();
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redAudienceFlowerIntake;
		}
	}

	private final class LeaveScoringFlowerState extends AbstractState {

		Path leaveFlower;

		public void init() {
			leaveFlower = Paths.line(lastPath, poses.redScoringFlowerLeave).constant(Math.toRadians(90));
			setEndCondition(() -> !follower.following());
			follower.follow(leaveFlower);
		}

		@Override
		public void loop() {
			follower.update();
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redAudienceFlowerLeave;
		}
	}

	private Submachine FlowerAudienceSub() {

		return new Submachine(
				"Flower", Arrays.asList(new ToAudienceFlowerState(), new WaitState(1), new LeaveAudienceFlowerState()));
	}

	private final class ToAudienceFlowerState extends AbstractState {

		Path goToFlower;
		Path leaveFlower;

		public void init() {
			if (lastPath.y() > 72) goToFlower = Paths.line(lastPath, poses.redAudienceFlowerIntake).constant(Math.toRadians(90));
			else goToFlower = Paths.curve(lastPath, poses.firstFlowerControl1, poses.redAudienceFlowerIntake)
					.linear(lastPath, poses.redAudienceFlowerIntake);
			setEndCondition(() -> !follower.following());
			follower.follow(goToFlower);
		}

		@Override
		public void loop() {
			follower.update();
		}

		@Override
		public void stop() {
			lastPath = poses.redAudienceFlowerIntake;
		}
	}

	private final class LeaveAudienceFlowerState extends AbstractState {

		Path leaveFlower;

		public void init() {
			leaveFlower = Paths.line(lastPath, poses.redScoringFlowerLeave).constant(Math.toRadians(90));
			setEndCondition(() -> !follower.following());
			follower.follow(leaveFlower);
		}

		@Override
		public void loop() {
			follower.update();
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redAudienceFlowerLeave;
		}
	}

	private Submachine GardenSub(boolean shooting) {
		List<State> garden = new ArrayList<>();
		garden.add(new GotoGarden());
		if (shooting) {
			garden.add(new ShootState("GardenShoot"));
		}
		garden.add(new RunIntake());
		garden.add(new RunTranferState());
		garden.add(new CollectGarden());
		garden.add(new StopTranferState());
		garden.add(new StopIntake());
		return new Submachine("Garden", garden);
	}

	private final class GotoGarden extends AbstractState {
		Path leavepath;

		@Override
		public void init() {
			leavepath = Paths.line(lastPath, poses.redToGarden).linear(lastPath, poses.redToGarden);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavepath);
		}

		@Override
		public void loop() {
			follower.update();
			telemetry.addData(
					"isdone?",
					Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			telemetry.addData("Velocity", follower.tangentialVelocity());
			telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
			telemetry.addData("distance to end", follower.distanceToEndpoint());
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redToGarden;
		}
	}

	private final class CollectGarden extends AbstractState {
		Path leavepath;

		@Override
		public void init() {
			leavepath = Paths.line(lastPath, poses.redPickupGarden).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavepath);
		}

		@Override
		public void loop() {
			follower.update();
			telemetry.addData(
					"isdone?",
					Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			telemetry.addData("Velocity", follower.tangentialVelocity());
			telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
			telemetry.addData("distance to end", follower.distanceToEndpoint());
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redPickupGarden;
		}
	}

	private final class ParkScoringState extends AbstractState {
		Path parkPath;

		@Override
		public void init() {
			parkPath = Paths.curve(lastPath, poses.redScoringParkControl, poses.redScoringPark).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(parkPath);
		}

		@Override
		public void loop() {
			follower.update();
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redScoringPark;
			follower.stop();
		}
	}

	private final class ParkAudienceState extends AbstractState {
		Path parkPath;

		@Override
		public void init() {
			parkPath = Paths.line(lastPath, poses.redAudiencePark).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(parkPath);
		}

		@Override
		public void loop() {
			follower.update();
			if (follower != null) Poses.finalPose = follower.pose();
		}

		@Override
		public void stop() {
			lastPath = poses.redAudiencePark;
			follower.stop();
		}
	}

	protected List<State> buildStates() {

		follower = Constants.create(safeMap);
		autoStates = new ArrayList<>();
		SetRoute();
		follower.setPose(getStartingPose());
		lastPath = getStartingPose();
		poses = new Poses(follower);

		// Same subsystems TeleOp uses, registered before any state runs.
		bulkCache = new BulkCache(orchestrator, safeMap, BulkCache.Mode.AUTO);
		orchestrator.registerNode("bulkcache", bulkCache);

		intake = safeMap.device(DcMotorEx.class, "intake");
		intakeNode = new Intake(orchestrator, intake);
		orchestrator.registerNode("intake", intakeNode);

		shooter = new Shooter(orchestrator, safeMap.device(DcMotorEx.class, "pollenShooter"), "g2");
		orchestrator.registerNode("shooter", shooter);

		shooterKicker = new ShooterKicker(orchestrator, safeMap.device(DcMotorEx.class, "shooterKicker"));
		orchestrator.registerNode("shooterKicker", shooterKicker);

		hood = new Hood(orchestrator, safeMap.device(Servo.class, "hoodServo"));
		orchestrator.registerNode("hood", hood);

		gate = new Gate(orchestrator, safeMap.device(Servo.class, "gate"));
		orchestrator.registerNode("gate", gate);

		// Auto drives the elevator through its override (see RunTranferState), like the Hood.
		elevator = new Elevator(orchestrator, safeMap.device(DcMotorEx.class, "elevator"));
		orchestrator.registerNode("elevator", elevator);

		// Turret shares the intake motor as its encoder, as in MainTeleOp, and aims from the follower.
		turret = new Turret(
				orchestrator,
				safeMap,
				safeMap.device(CRServo.class, "turretServo1"),
				safeMap.device(CRServo.class, "turretServo2"),
				safeMap.device(CRServo.class, "turretServo3"),
				intake,
				follower,
				"g2");
		orchestrator.registerNode("turret", turret);

		follower.holdEnd.set(true);

		return autoStates;
	}
}
