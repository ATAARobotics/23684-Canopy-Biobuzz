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
import org.firstinspires.ftc.teamcode.Subsystems.MatchGate;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class MainAuto extends StateMachineOpMode {

	// Where the Hood is parked once the match gate releases hardware writes, matching
	// MainTeleOp's HOOD_RELEASE_POSITION.
	private static final double HOOD_RELEASE_POSITION = 0.20;

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

	Elevator elevator;

	private Gate gate;
	private Intake intakeNode;

	public void setRoute() {}

	public void addStep(AutoStates state) {
		autoStates.add(getState(state));
	}

	public void addStep(int wait) {
		autoStates.add(new WaitState(wait));
	}

	protected abstract Pose getStartingPose();

	/** Which alliance this auto drives for; picks the side of the field the poses are laid out on. */
	protected abstract Team getTeam();

	private State getState(AutoStates state) {

		switch (state) {
			case shootPreload:
				return shootingSub();
			case leaveScoring:
				return new LeaveScoringState();
			case leaveAudience:
				return new LeaveAudienceState();
			case leavePushAlliance:
				return new LeavePushAlliance();
			case goToGarden:
				return gardenSub(false);
			case goToGardenShooting:
				return gardenSub(true);
			case pickFromFlowerScoring:
				return flowerScoringSub();
			case pickFromFlowerAudience:
				return flowerAudienceSub();
			case parkScoring:
				return new ParkScoringState();
			case parkAudience:
				return new ParkAudienceState();
			case shoot:
				return shootingSub();
		}
		return null;
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
		double speed = 0;

		@Override
		public void init() {
			// Shooting is done: close the Gate and let it follow the Shooter again.
			gate.setPowerOverride(false);
			// Hand the kicker back to the Shooter's RPM as well.
			shooterKicker.clearPowerOverride();
			setEndCondition(() -> Math.abs(speed) == 0);
		}

		@Override
		public void loop() {
			orchestrator.publish("shooter/targetRPM", 0.0);
			speed = orchestrator
					.getLatestValue("shooter/targetRPM", Double.class)
					.orElse(0.0);
		}
	}

	private Submachine shootingSub() {
		return new Submachine(
				"Shooter",
				Arrays.asList(
						new ShootState("shootingSub"),
						new RunTransferState(),
						new WaitState(0.2),
						new StopTransferState(),
						new ShooterStop()));
	}

	private final class RunTransferState extends AbstractState {
		float speed = 0f;

		@Override
		public void init() {
			// Drive the elevator directly instead of faking g2/right_stick_y: the override wins over
			// the auto-feed until StopTransferState clears it.
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

	private final class StopTransferState extends AbstractState {
		float speed = 0f;

		@Override
		public void init() {
			// Hand the elevator back to the auto-feed.
			elevator.setPowerOverride(0.0);
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
			orchestrator.publish("g2/left_stick_y", 0.0f);
			speed = orchestrator.getLatestValue("g2/left_stick_y", Float.class).orElse(0.0f);
		}
	}

	private final class LeaveScoringState extends AbstractState {
		Path leavePath;

		@Override
		public void init() {
			leavePath = Paths.line(lastPath, poses.scoringLeave).constant(lastPath);
			setEndCondition(() -> follower.atParametricEnd());
			follower.follow(leavePath);
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
			lastPath = poses.scoringLeave;
			follower.stop();
		}
	}

	private final class LeaveAudienceState extends AbstractState {
		Path leavePath;

		@Override
		public void init() {
			leavePath = Paths.line(lastPath, poses.audienceLeave).constant(lastPath);
			setEndCondition(() -> follower.atParametricEnd());
			follower.follow(leavePath);
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
			lastPath = poses.audienceLeave;
			follower.stop();
		}
	}

	private final class LeavePushAlliance extends AbstractState {
		Path leavePath;

		@Override
		public void init() {
			leavePath = Paths.line(lastPath, poses.parkingZoneAlliancePush).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavePath);
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
			lastPath = poses.parkingZoneAlliancePush;
			follower.stop();
		}
	}

	private Submachine flowerScoringSub() {

		return new Submachine(
				"Flower", Arrays.asList(new ToScoringFlowerState(), new WaitState(1), new LeaveScoringFlowerState()));
	}

	private final class ToScoringFlowerState extends AbstractState {

		Path goToFlower;
		Path leaveFlower;

		public void init() {
			if (lastPath.y() > 72) goToFlower = Paths.line(lastPath, poses.scoringFlowerIntake).constant(Math.toRadians(90));
			else goToFlower = Paths.curve(lastPath, poses.firstFlowerControl1, poses.scoringFlowerIntake)
					.linear(lastPath, poses.scoringFlowerIntake);
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
			lastPath = poses.scoringFlowerIntake;
			follower.stop();
		}
	}

	private final class LeaveScoringFlowerState extends AbstractState {

		Path leaveFlower;

		public void init() {
			leaveFlower = Paths.line(lastPath, poses.scoringFlowerLeave).constant(Math.toRadians(90));
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
			lastPath = poses.scoringFlowerLeave;
			follower.stop();
		}
	}

	private Submachine flowerAudienceSub() {

		return new Submachine(
				"Flower", Arrays.asList(new ToAudienceFlowerState(), new WaitState(1), new LeaveAudienceFlowerState()));
	}

	private final class ToAudienceFlowerState extends AbstractState {

		Path goToFlower;
		Path leaveFlower;

		public void init() {
			if (lastPath.y() > 72) goToFlower = Paths.curve(lastPath, poses.audienceFlowerIntake, poses.audienceFlowerIntake)
					.linear(lastPath, poses.audienceFlowerIntake);
			else goToFlower = Paths.curve(lastPath, poses.firstFlowerControl1, poses.audienceFlowerIntake)
					.linear(lastPath, poses.audienceFlowerIntake);
			setEndCondition(() -> !follower.following());
			follower.follow(goToFlower);
		}

		@Override
		public void loop() {
			follower.update();
		}

		@Override
		public void stop() {
			lastPath = poses.audienceFlowerIntake;
			follower.stop();
		}
	}

	private final class LeaveAudienceFlowerState extends AbstractState {

		Path leaveFlower;

		public void init() {
			leaveFlower = Paths.line(lastPath, poses.scoringFlowerLeave).constant(Math.toRadians(90));
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
			lastPath = poses.audienceFlowerLeave;
			follower.stop();
		}
	}

	private Submachine gardenSub(boolean shooting) {
		List<State> garden = new ArrayList<>();
		garden.add(new GotoGarden());
		if (shooting) {
			garden.add(new ShootState("GardenShoot"));
		}
		garden.add(new RunIntake());
		garden.add(new RunTransferState());
		garden.add(new CollectGarden());
		garden.add(new StopTransferState());
		garden.add(new StopIntake());
		return new Submachine("Garden", garden);
	}

	private final class GotoGarden extends AbstractState {
		Path leavePath;

		@Override
		public void init() {
			leavePath = Paths.line(lastPath, poses.toGarden).linear(lastPath, poses.toGarden);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavePath);
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
			lastPath = poses.toGarden;
			follower.stop();
		}
	}

	private final class CollectGarden extends AbstractState {
		Path leavePath;

		@Override
		public void init() {
			leavePath = Paths.line(lastPath, poses.pickupGarden).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(leavePath);
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
			lastPath = poses.pickupGarden;
			follower.stop();
		}
	}

	private final class ParkScoringState extends AbstractState {
		Path parkPath;

		@Override
		public void init() {
			parkPath = Paths.curve(lastPath, poses.scoringParkControl1, poses.scoringParkControl2, poses.scoringPark)
					.constant(lastPath);
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
			lastPath = poses.scoringPark;
			follower.stop();
		}
	}

	private final class ParkAudienceState extends AbstractState {
		Path parkPath;

		@Override
		public void init() {
			parkPath = Paths.curve(lastPath, poses.audienceParkControl, poses.audiencePark)
					.constant(lastPath);
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
			lastPath = poses.audiencePark;
			follower.stop();
		}
	}

	/**
	 * Auto has no gamepad to press the gate's release button, so release the writes as
	 * the route starts. Publishing the override's {@code false} value tells the
	 * {@link MatchGate} to let hardware writes flow for the rest of the routine.
	 */
	@Override
	protected void onRouteStarted() {
		orchestrator.publish(MatchGate.OVERRIDE_TOPIC, false);
	}

	protected List<State> buildStates() {

		follower = Constants.create(safeMap);
		poses = Poses.onSide(follower, getTeam());
		lastPath = getStartingPose();
		follower.setPose(lastPath);
		autoStates = new ArrayList<>();
		setRoute();

		// The gate holds hardware writes before any other subsystem attempts to run.
		orchestrator.registerNode("matchGate", new MatchGate(orchestrator));

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

		// Park the Hood at the same position TeleOp uses once the match gate releases
		// hardware writes, so auto starts from the same hood position.
		orchestrator.subscribe(MatchGate.HELD_TOPIC, Boolean.class, held -> {
			if (Boolean.FALSE.equals(held)) {
				hood.moveTo(HOOD_RELEASE_POSITION);
			}
		});

		gate = new Gate(orchestrator, safeMap.device(Servo.class, "gate"));
		orchestrator.registerNode("gate", gate);

		// Auto drives the elevator through its override (see RunTransferState), like the Hood.
		elevator = new Elevator(orchestrator, safeMap.device(DcMotorEx.class, "elevator"));
		orchestrator.registerNode("elevator", elevator);

		// The OpMode's alliance decides which HIVE cell the turret aims at, so the auto is right
		// without any Panels alliance switch being set to match.
		char allianceLetter = getTeam().allianceLetter();

		// Turret shares the intake motor as its encoder, as in MainTeleOp, and aims from the follower.
		turret = new Turret(
				orchestrator,
				safeMap,
				safeMap.device(CRServo.class, "turretServo1"),
				safeMap.device(CRServo.class, "turretServo2"),
				safeMap.device(CRServo.class, "turretServo3"),
				intake,
				follower,
				"g2",
				allianceLetter);
		orchestrator.registerNode("turret", turret);

		follower.holdEnd.set(true);

		return autoStates;
	}
}
