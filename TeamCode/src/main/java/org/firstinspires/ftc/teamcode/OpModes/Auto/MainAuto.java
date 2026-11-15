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
		wait
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
				return new ShootPreloadState();
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
		public void stop() {
			orchestrator.publish("shooter/targetRPM", 0.0);
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
			Poses.finalPose = follower.pose();
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
			Poses.finalPose = follower.pose();
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
			Poses.finalPose = follower.pose();
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
			Poses.finalPose = follower.pose();
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
			Poses.finalPose = follower.pose();
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
			Poses.finalPose = follower.pose();
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
		garden.add(new CollectGarden());
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
			Poses.finalPose = follower.pose();
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
			Poses.finalPose = follower.pose();
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
			parkPath = Paths.line(lastPath, poses.redScoringPark).constant(lastPath);
			setEndCondition(
					() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get()
							&& follower.distanceToEndpoint() < 4);
			follower.follow(parkPath);
		}

		@Override
		public void loop() {
			follower.update();
			Poses.finalPose = follower.pose();
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
			Poses.finalPose = follower.pose();
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
		orchestrator.registerNode("intake", new Intake(orchestrator, intake));

		shooter = new Shooter(orchestrator, safeMap.device(DcMotorEx.class, "pollenShooter"), "g2");
		orchestrator.registerNode("shooter", shooter);

		shooterKicker = new ShooterKicker(orchestrator, safeMap.device(DcMotorEx.class, "shooterKicker"));
		orchestrator.registerNode("shooterKicker", shooterKicker);

		hood = new Hood(orchestrator, safeMap.device(Servo.class, "hoodServo"));
		orchestrator.registerNode("hood", hood);

		// Turret shares the intake motor as its encoder, as in MainTeleOp, and aims from the follower.
		turret = new Turret(orchestrator, safeMap, safeMap.device(CRServo.class, "turret"), intake, follower, "g2");
		orchestrator.registerNode("turret", turret);

		follower.holdEnd.set(true);

		return autoStates;
	}
}
