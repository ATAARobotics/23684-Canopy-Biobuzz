package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.control.controller.ProfiledPIDController;
import com.aaravlabs.control.controller.SimpleMotorFeedforward;
import com.aaravlabs.control.trajectory.TrapezoidProfile;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilTagVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ClusterTarget;

/**
 * Turns the turret so the shooter stays aimed at our alliance's HIVE cell.
 *
 * <p>Angles are degrees from straight ahead, growing clockwise, so +15 is 15 degrees to the right.
 * The turret stops at the ends of its travel instead of pushing into them.
 *
 * <p>Where to aim comes from vision when a fresh cluster of ours is in view, otherwise from the
 * cell's fixed position on the field plus the robot's pose; {@link #setManualGoal(Double)} overrides
 * both for bench tuning. Movement is feedforward plus PID in volts (the WPILib pattern), and servo
 * power is a duty cycle rather than volts, so the volts get divided by the battery voltage.
 */
public class Turret extends Node {

	public static final String SOURCE_VISION = "VISION";
	public static final String SOURCE_LOCALIZATION = "LOCALIZATION";
	public static final String SOURCE_MANUAL = "MANUAL";
	public static final String SOURCE_OFF = "OFF";

	private static final double LOOP_PERIOD_SEC = 0.02;
	private static final double MIN_PROFILE_CONSTRAINT = 1e-3;
	/** Stop pushing once the turret is this close to a limit. */
	private static final double LIMIT_EPS_DEG = 0.5;
	/** Re-read the battery every 25 ticks (half a second) because the read is the slow part. */
	private static final int BATTERY_REFRESH_TICKS = 25;
	/** |error| samples behind the settle reading on the tuning screen. */
	private static final int SETTLE_WINDOW_SAMPLES = 25;

	private final SafeDevice<CRServo> turretServo;
	private final SafeDevice<DcMotorEx> encoderMotor;
	private final SafeHardwareMap safeMap;
	/** Null on the bench, where the tuning OpMode has no camera and no odometry. */
	private final Localization localization;
	/** Null on the bench, where the tuning OpMode has no camera and no odometry. */
	private final AprilTagVision vision;
	/** Gamepad that owns the offsets and the always-face toggle, "g1" or "g2". */
	private final String inputPrefix;

	private final ProfiledPIDController profiledPid;
	private final SimpleMotorFeedforward feedforward;
	private double constraintsV = -1.0;
	private double constraintsA = -1.0;
	private double prevSetpointVel;

	// Driver-set during a run, so instance state and not Panels statics, which outlive a match.
	private volatile double aimOffsetDeg;
	private volatile double lateralOffsetCm;
	private volatile boolean enabled = true;

	/** Bench override: an absolute angle to face instead of a cell. */
	private volatile Double manualGoal;

	// Encoder zero, taken on the first tick, which is the turret facing straight ahead at init.
	private boolean zeroCaptured;
	private int zeroTicks;

	// Whether the previous tick had something to aim at. Re-acquiring a target re-seeds the profile.
	private boolean hadTarget;

	private int batteryTick;
	private double batteryVoltage = TurretConfig.nominalVoltage;

	// Peak and settle error per step cycle, latched when the goal changes so the tuning screen
	// reports the numbers of the cycle that has just finished.
	private final SettleWindow settleWindow = new SettleWindow();
	private boolean goalInitialized;
	private double lastGoal;
	private double cyclePeakErrorDeg;
	private volatile double lastCyclePeakErrorDeg;
	private volatile double lastCycleSettleErrorDeg;

	// Loop state snapshots, published for telemetry and read by the getters.
	private volatile double angleDeg;
	private volatile double aimDeg;
	private volatile double goalDeg;
	private volatile double errorDeg;
	private volatile double setpointDeg;
	private volatile double setpointVelDegPerSec;
	private volatile double ffVolts;
	private volatile double pidVolts;
	private volatile double power;
	private volatile boolean atLimit;
	private volatile int encoderTicks;
	private volatile String source = SOURCE_OFF;
	private volatile String targetName = "—";
	private volatile boolean targetStale;

	public Turret(
			Orchestrator orchestrator,
			SafeHardwareMap safeMap,
			SafeDevice<CRServo> turretServo,
			SafeDevice<DcMotorEx> encoderMotor,
			Localization localization,
			AprilTagVision vision,
			String inputPrefix) {
		super(orchestrator);
		this.safeMap = safeMap;
		this.turretServo = turretServo;
		this.encoderMotor = encoderMotor;
		this.localization = localization;
		this.vision = vision;
		this.inputPrefix = inputPrefix;

		this.profiledPid = new ProfiledPIDController(
				0.0,
				0.0,
				0.0,
				new TrapezoidProfile.Constraints(
						Math.max(TurretConfig.maxVelocityDegPerSec, MIN_PROFILE_CONSTRAINT),
						Math.max(TurretConfig.maxAccelerationDegPerSec2, MIN_PROFILE_CONSTRAINT)),
				LOOP_PERIOD_SEC);
		this.feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0, LOOP_PERIOD_SEC);
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void controlLoop() {
		double angle = measureAngle();

		Aim aim = computeAim(angle);
		boolean hasTarget = aim != null;
		double goal = hasTarget ? track(angle, aim) : holdPosition(angle);
		hadTarget = hasTarget;
		if (hasTarget) {
			recordCycle(goal);
		}

		refreshBattery();
		double power = enabled && hasTarget ? toPower(ffVolts + pidVolts) : 0.0;

		atLimit = false;
		if (angle >= TurretConfig.maxAngleDeg - LIMIT_EPS_DEG && power > 0.0) {
			power = 0.0;
			atLimit = true;
		} else if (angle <= TurretConfig.minAngleDeg + LIMIT_EPS_DEG && power < 0.0) {
			power = 0.0;
			atLimit = true;
		}
		turretServo.raw().setPower(power);

		publish(angle, goal, aim, power);
	}

	/**
	 * Encoder ticks since the zero captured on the first tick, as degrees. The turret points
	 * straight ahead at init, so that zero is forward.
	 */
	private double measureAngle() {
		int ticks = encoderMotor.raw().getCurrentPosition();
		if (!zeroCaptured) {
			zeroTicks = ticks;
			zeroCaptured = true;
		}
		encoderTicks = ticks - zeroTicks;
		double sign = TurretConfig.encoderReversed ? -1.0 : 1.0;
		return encoderTicks * sign / TurretConfig.countsPerTurretDegree();
	}

	/**
	 * Nothing to aim at, so hold still and re-seed the profile from the measurement. Without this
	 * the next target would start its move from a stale setpoint.
	 */
	private double holdPosition(double angle) {
		profiledPid.reset(angle);
		prevSetpointVel = 0.0;
		ffVolts = 0.0;
		pidVolts = 0.0;
		setpointDeg = angle;
		setpointVelDegPerSec = 0.0;
		errorDeg = 0.0;
		return angle;
	}

	/**
	 * Pick the goal inside the travel range, then advance the trapezoid profile and the PID by one
	 * tick. Re-seed from the measurement on the first tick and whenever a target is re-acquired.
	 */
	private double track(double angle, Aim aim) {
		double goal = selectGoal(aim.angleDeg, angle);
		if (!goalInitialized || !hadTarget) {
			profiledPid.reset(angle);
			prevSetpointVel = 0.0;
		}
		refreshController();
		profiledPid.setGoal(goal);
		pidVolts = profiledPid.calculate(angle);
		TrapezoidProfile.State setpoint = profiledPid.getSetpoint();
		ffVolts = feedforward.calculate(prevSetpointVel, setpoint.velocity);
		prevSetpointVel = setpoint.velocity;
		setpointDeg = setpoint.position;
		setpointVelDegPerSec = setpoint.velocity;
		errorDeg = profiledPid.getPositionError();
		return goal;
	}

	/** Controller volts to servo duty cycle. */
	private double toPower(double volts) {
		double power = TurretConfig.powerSign * volts / Math.max(batteryVoltage, 1e-6);
		return clamp(power, -1.0, 1.0);
	}

	private void refreshBattery() {
		batteryTick--;
		if (batteryTick <= 0) {
			batteryTick = BATTERY_REFRESH_TICKS;
			batteryVoltage = readBatteryVoltage();
		}
	}

	/** Keep the peak and settle error of the current cycle until the goal moves on. */
	private void recordCycle(double goal) {
		double absError = Math.abs(errorDeg);
		if (absError > cyclePeakErrorDeg) {
			cyclePeakErrorDeg = absError;
		}
		settleWindow.add(absError);

		if (!goalInitialized) {
			lastGoal = goal;
			goalInitialized = true;
		} else if (Math.abs(goal - lastGoal) > 1e-6) {
			lastCyclePeakErrorDeg = cyclePeakErrorDeg;
			lastCycleSettleErrorDeg = settleWindow.mean();
			cyclePeakErrorDeg = 0.0;
			lastGoal = goal;
		}
	}

	private void publish(double angle, double goal, Aim aim, double servoPower) {
		boolean hasTarget = aim != null;

		source = hasTarget && enabled ? aim.source : SOURCE_OFF;
		targetName = hasTarget ? aim.targetName : "—";
		targetStale = hasTarget && aim.stale;
		angleDeg = angle;
		aimDeg = hasTarget ? aim.angleDeg : angle;
		goalDeg = goal;
		power = servoPower;

		orchestrator.publish("turret/angle", angle);
		orchestrator.publish("turret/goal", goal);
		orchestrator.publish("turret/error", errorDeg);
		orchestrator.publish("turret/ffVolts", ffVolts);
		orchestrator.publish("turret/pidVolts", pidVolts);
		orchestrator.publish("turret/power", servoPower);
		orchestrator.publish("turret/source", source);
		orchestrator.publish("turret/targetName", targetName);
		orchestrator.publish("turret/offsetAimDeg", aimOffsetDeg);
		orchestrator.publish("turret/offsetLateralCm", lateralOffsetCm);
		orchestrator.publish("turret/atLimit", atLimit);
		orchestrator.publish("turret/enabled", enabled);

		// The camera rides the turret, so the vision node cannot resolve geometry without our yaw.
		// Its convention grows counter-clockwise and ours grows clockwise, hence the minus sign.
		orchestrator.publish("vision/setCameraYaw", -(angle + TurretConfig.cameraMountYawDeg));
	}

	// ── aiming ────────────────────────────────────────────────────────────────────

	/** What the turret should be facing this tick. */
	private static final class Aim {
		/** Angle to point at, clockwise from forward, in the same frame as the turret angle. */
		final double angleDeg;
		final String source;
		final String targetName;
		/** Horizontal distance to the aim point in cm, or −1 when there is nothing to scale with. */
		final double horizontalRangeCm;
		final boolean stale;

		Aim(double angleDeg, String source, String targetName, double horizontalRangeCm, boolean stale) {
			this.angleDeg = angleDeg;
			this.source = source;
			this.targetName = targetName;
			this.horizontalRangeCm = horizontalRangeCm;
			this.stale = stale;
		}
	}

	/** Where the turret should point now, or null when there is nothing to aim at. */
	private Aim computeAim(double angle) {
		Double manual = manualGoal;
		if (manual != null) {
			// Bench override: an absolute angle, so there is no range for the lateral offset to
			// scale against and only the aim offset applies.
			return new Aim(
					manual + aimOffsetDeg,
					SOURCE_MANUAL,
					String.format(Locale.US, "MANUAL(%.1f)", manual),
					-1.0,
					false);
		}

		ClusterTarget cluster = bestCluster();
		if (cluster != null) {
			// The camera rides the turret, so its bearing to the cluster is the mount offset plus
			// the bearing it reports. The SDK reports bearing positive-left, hence the minus sign.
			double bearing = TurretConfig.cameraMountYawDeg - cluster.rawBearing;
			double rangeCm = cluster.resolved.horizontalDistance * 2.54;
			return new Aim(
					angle + bearing + aimOffsetDeg + lateralShift(rangeCm),
					SOURCE_VISION,
					cluster.name,
					rangeCm,
					cluster.stale);
		}

		double[] cell = cellPosition();
		Pose pose = localization == null ? null : localization.getPose();
		if (cell != null && pose != null) {
			double dx = cell[0] - pose.x();
			double dy = cell[1] - pose.y();
			// Minus sign as above: Pedro's heading grows counter-clockwise, ours grows clockwise.
			double bearing = wrapDeg(Math.toDegrees(pose.heading() - Math.atan2(dy, dx)));
			double rangeCm = Math.hypot(dx, dy) * 2.54;
			return new Aim(
					angle + bearing + aimOffsetDeg + lateralShift(rangeCm),
					SOURCE_LOCALIZATION,
					String.format(Locale.US, "CELL(%.1f, %.1f)", cell[0], cell[1]),
					rangeCm,
					false);
		}
		return null;
	}

	/**
	 * Extra angle for a shooter that sits off the turret axis: the lateral offset seen from the
	 * target's range. Positive moves the aim right of the target.
	 */
	private double lateralShift(double rangeCm) {
		if (rangeCm <= 1e-6) {
			return 0.0;
		}
		return Math.toDegrees(Math.atan2(lateralOffsetCm, rangeCm));
	}

	/** Nearest fresh cluster that is ours and on our half of the field. */
	private ClusterTarget bestCluster() {
		if (vision == null) {
			return null;
		}
		List<ClusterTarget> clusters = vision.getTargets();
		if (clusters == null) {
			return null;
		}
		char letter = TurretConfig.allianceLetter();
		String half = fieldHalf();
		ClusterTarget best = null;
		for (ClusterTarget cluster : clusters) {
			if (cluster == null || cluster.stale || cluster.name == null || cluster.name.isEmpty()) {
				continue;
			}
			if (Character.toUpperCase(cluster.name.charAt(0)) != letter) {
				continue;
			}
			if (half != null && !cluster.name.toUpperCase(Locale.US).contains(half)) {
				continue;
			}
			if (best == null || cluster.resolved.range < best.resolved.range) {
				best = cluster;
			}
		}
		return best;
	}

	/** "SCORING" or "AUDIENCE" for the half of the field the robot is on, or null if unknown. */
	private String fieldHalf() {
		if (localization == null) {
			return null;
		}
		Pose pose = localization.getPose();
		if (pose == null) {
			return null;
		}
		return pose.y() > TurretConfig.halfSplitY ? "SCORING" : "AUDIENCE";
	}

	/** Fixed field position of our HIVE cell, in PedroPathing inches, or null if unknown. */
	private double[] cellPosition() {
		String half = fieldHalf();
		if (half == null) {
			return null;
		}
		boolean blue = TurretConfig.allianceLetter() == 'B';
		boolean scoring = "SCORING".equals(half);
		if (!blue && !scoring) {
			return new double[] {TurretConfig.cellRedAudienceX, TurretConfig.cellRedAudienceY};
		}
		if (!blue) {
			return new double[] {TurretConfig.cellRedScoringX, TurretConfig.cellRedScoringY};
		}
		if (!scoring) {
			return new double[] {TurretConfig.cellBlueAudienceX, TurretConfig.cellBlueAudienceY};
		}
		return new double[] {TurretConfig.cellBlueScoringX, TurretConfig.cellBlueScoringY};
	}

	/**
	 * Of the wanted angle and the same angle plus or minus a whole turn, pick the one that lands
	 * inside the travel range and is closest to where the turret is now. This is what stops a
	 * "turn 20 degrees left" command from being sent the long way round.
	 */
	private static double selectGoal(double angleWanted, double currentAngle) {
		double min = TurretConfig.minAngleDeg;
		double max = TurretConfig.maxAngleDeg;
		double best = clamp(angleWanted, min, max);
		double bestDistance = Double.POSITIVE_INFINITY;
		for (int k = -1; k <= 1; k++) {
			double candidate = angleWanted + 360.0 * k;
			if (candidate < min || candidate > max) {
				continue;
			}
			double distance = Math.abs(candidate - currentAngle);
			if (distance < bestDistance) {
				bestDistance = distance;
				best = candidate;
			}
		}
		return best;
	}

	// ── gamepad ───────────────────────────────────────────────────────────────────
	// Dpad left/right trims the aim angle and up/down trims the lateral offset, y zeroes both and
	// x toggles the turret. There are two handlers per button only because each gamepad publishes
	// on its own topic prefix; the guard keeps the turret on the one it was built with.

	@SubscribedTo(topic = "g1/dpad_left/rising")
	public void g1DpadLeft() {
		if ("g1".equals(inputPrefix)) {
			nudgeAim(-1.0);
		}
	}

	@SubscribedTo(topic = "g2/dpad_left/rising")
	public void g2DpadLeft() {
		if ("g2".equals(inputPrefix)) {
			nudgeAim(-1.0);
		}
	}

	@SubscribedTo(topic = "g1/dpad_right/rising")
	public void g1DpadRight() {
		if ("g1".equals(inputPrefix)) {
			nudgeAim(1.0);
		}
	}

	@SubscribedTo(topic = "g2/dpad_right/rising")
	public void g2DpadRight() {
		if ("g2".equals(inputPrefix)) {
			nudgeAim(1.0);
		}
	}

	/** One degree per press, or a tenth of a degree while the left bumper is held. */
	private double stepSize() {
		boolean bumper = orchestrator
				.getLatestValue(inputPrefix + "/left_bumper", Boolean.class)
				.orElse(false);
		return bumper ? 0.1 : 1.0;
	}

	@SubscribedTo(topic = "g1/dpad_up/rising")
	public void g1DpadUp() {
		if ("g1".equals(inputPrefix)) {
			nudgeLateral(1.0);
		}
	}

	@SubscribedTo(topic = "g2/dpad_up/rising")
	public void g2DpadUp() {
		if ("g2".equals(inputPrefix)) {
			nudgeLateral(1.0);
		}
	}

	@SubscribedTo(topic = "g1/dpad_down/rising")
	public void g1DpadDown() {
		if ("g1".equals(inputPrefix)) {
			nudgeLateral(-1.0);
		}
	}

	@SubscribedTo(topic = "g2/dpad_down/rising")
	public void g2DpadDown() {
		if ("g2".equals(inputPrefix)) {
			nudgeLateral(-1.0);
		}
	}

	@SubscribedTo(topic = "g1/y/rising")
	public void g1ZeroOffsets() {
		if ("g1".equals(inputPrefix)) {
			aimOffsetDeg = 0.0;
			lateralOffsetCm = 0.0;
		}
	}

	@SubscribedTo(topic = "g2/y/rising")
	public void g2ZeroOffsets() {
		if ("g2".equals(inputPrefix)) {
			aimOffsetDeg = 0.0;
			lateralOffsetCm = 0.0;
		}
	}

	@SubscribedTo(topic = "g1/x/rising")
	public void g1ToggleEnabled() {
		if ("g1".equals(inputPrefix)) {
			enabled = !enabled;
		}
	}

	@SubscribedTo(topic = "g2/x/rising")
	public void g2ToggleEnabled() {
		if ("g2".equals(inputPrefix)) {
			enabled = !enabled;
		}
	}

	private void nudgeAim(double direction) {
		aimOffsetDeg += direction * stepSize();
	}

	private void nudgeLateral(double direction) {
		lateralOffsetCm += direction * stepSize();
	}

	// ── helpers ───────────────────────────────────────────────────────────────────

	/** Push the live Panels gains and limits into the controller so edits land immediately. */
	private void refreshController() {
		profiledPid.setPID(nonNegative(TurretConfig.kp), nonNegative(TurretConfig.ki), nonNegative(TurretConfig.kd));
		profiledPid.setIZone(Math.max(TurretConfig.iZone, 0.0));
		profiledPid.setIntegratorRange(TurretConfig.integratorMin, TurretConfig.integratorMax);
		profiledPid.setTolerance(Math.max(TurretConfig.positionToleranceDeg, 0.0));

		double v = Math.max(TurretConfig.maxVelocityDegPerSec, MIN_PROFILE_CONSTRAINT);
		double a = Math.max(TurretConfig.maxAccelerationDegPerSec2, MIN_PROFILE_CONSTRAINT);
		if (v != constraintsV || a != constraintsA) {
			profiledPid.setConstraints(new TrapezoidProfile.Constraints(v, a));
			constraintsV = v;
			constraintsA = a;
		}

		feedforward.setKs(TurretConfig.ks);
		feedforward.setKv(nonNegative(TurretConfig.kv));
		feedforward.setKa(nonNegative(TurretConfig.ka));
	}

	/** First positive reading on the bus, or the configured nominal voltage if none are. */
	private double readBatteryVoltage() {
		try {
			for (VoltageSensor sensor : safeMap.raw().getAll(VoltageSensor.class)) {
				double volts = sensor.getVoltage();
				if (volts > 0.0) {
					return volts;
				}
			}
		} catch (Exception e) {
			orchestrator.warn("battery voltage read failed: " + e.getMessage());
		}
		return TurretConfig.nominalVoltage > 0.0 ? TurretConfig.nominalVoltage : 12.0;
	}

	private static double nonNegative(double value) {
		return Double.isNaN(value) ? 0.0 : Math.max(value, 0.0);
	}

	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(value, max));
	}

	private static double wrapDeg(double degrees) {
		double d = degrees % 360.0;
		if (d > 180.0) {
			d -= 360.0;
		} else if (d <= -180.0) {
			d += 360.0;
		}
		return d;
	}

	// ── tuning API & telemetry ────────────────────────────────────────────────────

	/** Bench override: face this absolute angle instead of a cell. Null returns to the cells. */
	public void setManualGoal(Double goal) {
		manualGoal = goal;
	}

	public double getAngleDeg() {
		return angleDeg;
	}

	public double getAimDeg() {
		return aimDeg;
	}

	public double getGoalDeg() {
		return goalDeg;
	}

	public double getSetpointDeg() {
		return setpointDeg;
	}

	public double getSetpointVelocityDegPerSec() {
		return setpointVelDegPerSec;
	}

	public double getErrorDeg() {
		return errorDeg;
	}

	public double getFfVolts() {
		return ffVolts;
	}

	public double getPidVolts() {
		return pidVolts;
	}

	public double getPower() {
		return power;
	}

	public double getBatteryVoltage() {
		return batteryVoltage;
	}

	public int getEncoderTicks() {
		return encoderTicks;
	}

	public double getLastCyclePeakErrorDeg() {
		return lastCyclePeakErrorDeg;
	}

	public double getLastCycleSettleErrorDeg() {
		return lastCycleSettleErrorDeg;
	}

	public double getAimOffsetDeg() {
		return aimOffsetDeg;
	}

	public double getLateralOffsetCm() {
		return lateralOffsetCm;
	}

	public boolean isEnabled() {
		return enabled;
	}

	/** Primary driver-station line. The extra detail goes below it via {@link #publishVerboseTelemetry}. */
	public void publishTelemetry(Telemetry telemetry) {
		telemetry.addData(
				"Turret",
				"%s → %s   aim %.1f°  goal %.1f°  err %+.1f°  pow %+.2f   offs %+.1f°/%+.1fcm   V-%s",
				source,
				targetName,
				aimDeg,
				goalDeg,
				errorDeg,
				power,
				aimOffsetDeg,
				lateralOffsetCm,
				enabled
						? "ON"
						: "OFF");
	}

	/** Extra detail, only shown when {@link TurretConfig#verboseTelemetry} is on. */
	public void publishVerboseTelemetry(Telemetry telemetry) {
		telemetry.addData(
				"Turret ctrl",
				"ff %+.2fV  pid %+.2fV  setpoint %.1f° @ %.0f°/s  battery %.1fV  stale %s%s",
				ffVolts,
				pidVolts,
				setpointDeg,
				setpointVelDegPerSec,
				batteryVoltage,
				targetStale
						? "yes"
						: "no",
				atLimit
						? "  AT LIMIT"
						: "");
		if (localization != null) {
			Pose pose = localization.getPose();
			if (pose != null) {
				telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
			}
		}
	}

	@Override
	public void close() {
		turretServo.run(m -> m.setPower(0.0));
	}

	/** Running average of the most recent |error| samples. */
	private static final class SettleWindow {
		private final double[] samples = new double[SETTLE_WINDOW_SAMPLES];
		private int index;
		private int count;

		void add(double value) {
			samples[index] = value;
			index = (index + 1) % samples.length;
			if (count < samples.length) {
				count++;
			}
		}

		double mean() {
			if (count == 0) {
				return 0.0;
			}
			double sum = 0.0;
			for (int i = 0; i < count; i++) {
				sum += samples[i];
			}
			return sum / count;
		}
	}
}
