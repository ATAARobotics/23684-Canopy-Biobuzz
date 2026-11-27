package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.AutoContext;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.FlowerAudienceSub;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.FlowerScoringSub;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.FollowPathState;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.FollowPathState.EndStrategy;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.GardenSub;
import org.firstinspires.ftc.teamcode.OpModes.Auto.States.ShootingSub;
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

/**
 * Base OpMode for the autonomous routines: builds the subsystems the states drive and turns a
 * {@link AutoStates} step into the state that runs it. The states themselves live in
 * {@link org.firstinspires.ftc.teamcode.OpModes.Auto.States}; subclasses supply the starting pose,
 * alliance, and ordered route via {@link #setRoute()}.
 */
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

	Poses poses;

	/** Everything the states share: follower, poses, orchestrator, telemetry and the pose cursor. */
	private AutoContext ctx;

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
				return new FollowPathState(
						ctx,
						"LeaveScoringState",
						EndStrategy.AT_PARAMETRIC_END,
						start -> Paths.line(start, ctx.poses().scoringLeave).constant(start));
			case leaveAudience:
				return new FollowPathState(
						ctx,
						"LeaveAudienceState",
						EndStrategy.AT_PARAMETRIC_END,
						start -> Paths.line(start, ctx.poses().audienceLeave).constant(start));
			case leavePushAlliance:
				return new FollowPathState(
						ctx,
						"LeavePushAlliance",
						EndStrategy.SETTLED,
						start -> Paths.line(start, ctx.poses().parkingZoneAlliancePush).constant(start));
			case goToGarden:
				return gardenSub(false);
			case goToGardenShooting:
				return gardenSub(true);
			case pickFromFlowerScoring:
				return flowerScoringSub();
			case pickFromFlowerAudience:
				return flowerAudienceSub();
			case parkScoring:
				return new FollowPathState(
						ctx,
						"ParkScoringState",
						EndStrategy.SETTLED,
						start -> Paths.curve(
								start,
								ctx.poses().scoringParkControl1,
								ctx.poses().scoringParkControl2,
								ctx.poses().scoringPark)
								.constant(start));
			case parkAudience:
				return new FollowPathState(
						ctx,
						"ParkAudienceState",
						EndStrategy.SETTLED,
						start -> Paths.curve(start, ctx.poses().audienceParkControl, ctx.poses().audiencePark)
								.constant(start));
			case shoot:
				return shootingSub();
		}
		return null;
	}

	private Submachine shootingSub() {
		return new ShootingSub(ctx, gate, shooterKicker, elevator);
	}

	private Submachine gardenSub(boolean shooting) {
		return new GardenSub(ctx, gate, shooterKicker, intakeNode, elevator, shooting);
	}

	private Submachine flowerScoringSub() {
		return new FlowerScoringSub(ctx);
	}

	private Submachine flowerAudienceSub() {
		return new FlowerAudienceSub(ctx);
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
		Pose lastPath = getStartingPose();
		follower.setPose(lastPath);
		autoStates = new ArrayList<>();
		ctx = new AutoContext(follower, poses, orchestrator, telemetry, lastPath);

		// The gate holds hardware writes before any other subsystem attempts to run.
		orchestrator.registerNode("matchGate", new MatchGate(orchestrator));

		// Same subsystems TeleOp uses, registered before any state runs.
		bulkCache = new BulkCache(orchestrator, safeMap, BulkCache.Mode.AUTO);
		orchestrator.registerNode("bulkcache", bulkCache);

		SafeDevice<DcMotorEx> intake = safeMap.device(DcMotorEx.class, "intake");
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

		// Auto drives the elevator through its override (see PowerOverrideState), like the Hood.
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
		TurretConfig.encoderReversed = false;

		setRoute();

		follower.holdEnd.set(true);

		return autoStates;
	}
}
