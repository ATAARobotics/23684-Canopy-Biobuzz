package org.firstinspires.ftc.teamcode.Subsystems;

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
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Keeps the turret facing the correct HIVE CELL at all times.
 *
 * <p>Aim target: the AprilTag cluster for our alliance and the half of the field the robot is
 * physically on (vision), falling back to the cell's fixed field position and PedroPathing
 * localization when no such cluster is visible. Control is feedforward carrying the trapezoid
 * profile with a small PID cleaning up residual error (WPILib pattern).
 *
 * <p>Frames and signs: turret angle θ is CCW-positive degrees from robot forward, measured {@code θ
 * = (ticks − zeroTicks) × encoderReversedSign ÷ countsPerTurretDegree} with the ratio derived from
 * the mechanical parameters in {@link TurretConfig}. The encoder zero is captured on the first
 * hardware tick (OpMode init) with the turret facing forward.
 *
 * <p>Threading: the control loop runs on the single hardware thread ({@code hardware = true}) and
 * touches hardware only through {@code SafeDevice.raw()}; gamepad handlers run on the callback pool
 * and only flip volatile fields.
 */
public class Turret extends Node {

	public static final String SOURCE_VISION = "VISION";
	public static final String SOURCE_LOCALIZATION = "LOCALIZATION";
	public static final String SOURCE_MANUAL = "MANUAL";
	public static final String SOURCE_OFF = "OFF";

	private static final double CONTROLLER_PERIOD_SEC = 0.02;
	private static final double MIN_CONSTRAINT = 1e-3;
	private static final double LIMIT_EPS_DEG = 0.5;
	private static final int BATTERY_REFRESH_TICKS = 25;
	/** Sliding window of |error| covering the final 0.5 s of a step cycle (25 ticks at 50 Hz). */
	private static final int SETTLE_WINDOW_SAMPLES = 25;

	private final SafeDevice<CRServo> turretServo;
	private final SafeDevice<DcMotorEx> encoderMotor;
	private final SafeHardwareMap safeMap;
	/** Nullable: bench tuning runs without vision/localization (manual goal only). */
	private final Localization localization;
	/** Nullable: bench tuning runs without vision/localization (manual goal only). */
	private final AprilTagVision vision;
	/** Gamepad prefix that owns the offsets and the always-face toggle ("g1" or "g2"). */
	private final String inputPrefix;

	private final ProfiledPIDController profiledPid;
	private final SimpleMotorFeedforward feedforward;
	private double constraintsV = -1.0;
	private double constraintsA = -1.0;
	private double prevSetpointVel;

	// Per-run aim offsets — gamepad state, zeroed at init. Deliberately NOT Panels Configurables:
	// Panels statics persist across OpMode runs.
	private volatile double aimOffsetDeg;
	private volatile double lateralOffsetCm;
	private volatile boolean enabled = true;

	/** Tuning override: when non-null, face this absolute aim angle instead of a cell. */
	private volatile Double manualGoal;

	// Encoder zero, captured on the first hardware tick (§3.9: turret faces forward at init).
	private boolean zeroCaptured;
	private int zeroTicks;
	private boolean hadTarget;

	private int batteryTick;
	private double batteryVoltage = TurretConfig.nominalVoltage;

	// Step-cycle statistics, computed here and latched whenever the goal changes (Tuning screen).
	private final SettleWindow settleWindow = new SettleWindow();
	private boolean goalInitialized;
	private double lastGoal;
	private double cyclePeakErrorDeg;
	private volatile double lastCyclePeakErrorDeg;
	private volatile double lastCycleSettleErrorDeg;

	// Snapshots for telemetry rendering.
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
						Math.max(TurretConfig.maxVelocityDegPerSec, MIN_CONSTRAINT),
						Math.max(TurretConfig.maxAccelerationDegPerSec2, MIN_CONSTRAINT)),
				CONTROLLER_PERIOD_SEC);
		this.feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0, CONTROLLER_PERIOD_SEC);
	}

	/**
	 * 50 Hz control loop on the hardware thread (§4.4): measure θ, pick the aim target, select the
	 * shortest-path goal inside the travel range, then feedforward (carrying the profile) + PID.
	 */
	@RunPeriodically(hz = 50, hardware = true)
	public void controlLoop() {
		// ── measure ─────────────────────────────────────────────────────────────
		int ticks = encoderMotor.raw().getCurrentPosition();
		if (!zeroCaptured) {
			zeroTicks = ticks;
			zeroCaptured = true;
		}
		encoderTicks = ticks - zeroTicks;
		double encoderSign = TurretConfig.encoderReversed ? -1.0 : 1.0;
		double angle = (ticks - zeroTicks) * encoderSign / TurretConfig.countsPerTurretDegree();

		// ── aim ─────────────────────────────────────────────────────────────────
		Aim aim = computeAim(angle);
		boolean haveTarget = aim != null;

		double goal;
		double currentFf = 0.0;
		double currentPid = 0.0;
		if (haveTarget) {
			if (!goalInitialized || !hadTarget) {
				// First tick or re-acquisition: seed the profile where we actually are.
				profiledPid.reset(angle);
				prevSetpointVel = 0.0;
			}
			goal = selectGoal(aim.alphaDeg, angle); // §4.3: anti-thrash, plain bounded problem
			refreshController();
			profiledPid.setGoal(goal);
			currentPid = profiledPid.calculate(angle);
			TrapezoidProfile.State setpoint = profiledPid.getSetpoint();
			currentFf = feedforward.calculate(prevSetpointVel, setpoint.velocity);
			prevSetpointVel = setpoint.velocity;
			setpointDeg = setpoint.position;
			setpointVelDegPerSec = setpoint.velocity;
			errorDeg = profiledPid.getPositionError();
		} else {
			// Nothing to face: hold position and keep the profile seeded from the measurement.
			goal = angle;
			profiledPid.reset(angle);
			prevSetpointVel = 0.0;
			setpointDeg = angle;
			setpointVelDegPerSec = 0.0;
			errorDeg = 0.0;
		}
		hadTarget = haveTarget;

		// Step-cycle stats (Tuning): latch when the goal moves, then keep accumulating.
		if (haveTarget) {
			if (!goalInitialized) {
				lastGoal = goal;
				goalInitialized = true;
			} else if (Math.abs(goal - lastGoal) > 1e-6) {
				lastCyclePeakErrorDeg = cyclePeakErrorDeg;
				lastCycleSettleErrorDeg = settleWindow.mean();
				cyclePeakErrorDeg = 0.0;
				lastGoal = goal;
			}
			double absError = Math.abs(errorDeg);
			if (absError > cyclePeakErrorDeg) {
				cyclePeakErrorDeg = absError;
			}
			settleWindow.add(absError);
		}

		// ── output ──────────────────────────────────────────────────────────────
		if (batteryTick <= 0) {
			batteryTick = BATTERY_REFRESH_TICKS;
			batteryVoltage = readBatteryVoltage();
		}
		batteryTick--;

		boolean active = enabled && haveTarget;
		double output = 0.0;
		if (active) {
			output = TurretConfig.powerSign * (currentFf + currentPid) / Math.max(batteryVoltage, 1e-6);
			output = clamp(output, -1.0, 1.0);
		}

		// Limit safety (§4.3): at a travel limit, cut power pushing further out.
		boolean limitHit = false;
		if (angle >= TurretConfig.maxAngleDeg - LIMIT_EPS_DEG && output > 0.0) {
			output = 0.0;
			limitHit = true;
		}
		if (angle <= TurretConfig.minAngleDeg + LIMIT_EPS_DEG && output < 0.0) {
			output = 0.0;
			limitHit = true;
		}
		turretServo.raw().setPower(output);

		// ── publish ─────────────────────────────────────────────────────────────
		String sourceLabel = active ? aim.source : SOURCE_OFF;
		String nameLabel = haveTarget ? aim.targetName : "—";

		angleDeg = angle;
		aimDeg = haveTarget ? aim.alphaDeg : angle;
		goalDeg = goal;
		ffVolts = currentFf;
		pidVolts = currentPid;
		power = output;
		atLimit = limitHit;
		source = sourceLabel;
		targetName = nameLabel;
		targetStale = haveTarget && aim.stale;

		orchestrator.publish("turret/angle", angle);
		orchestrator.publish("turret/goal", goal);
		orchestrator.publish("turret/error", errorDeg);
		orchestrator.publish("turret/ffVolts", currentFf);
		orchestrator.publish("turret/pidVolts", currentPid);
		orchestrator.publish("turret/power", output);
		orchestrator.publish("turret/source", sourceLabel);
		orchestrator.publish("turret/targetName", nameLabel);
		orchestrator.publish("turret/offsetAimDeg", aimOffsetDeg);
		orchestrator.publish("turret/offsetLateralCm", lateralOffsetCm);
		orchestrator.publish("turret/atLimit", limitHit);
		orchestrator.publish("turret/enabled", enabled);
		// Telemetry-only turret yaw sync for the vision node's resolved.* geometry (§4.5). The camera
		// rides the turret, so its yaw relative to the robot is the turret angle plus the mount
		// offset — the processor is built at yaw 0 and cannot be re-aimed, so this is what corrects
		// for it. Matches the α = θ + cameraMountYawDeg + β used in computeAim.
		orchestrator.publish("vision/setCameraYaw", angle + TurretConfig.cameraMountYawDeg);
	}

	// ── aiming ────────────────────────────────────────────────────────────────────

	/** Immutable snapshot of what the turret should face this tick. */
	private static final class Aim {
		final double alphaDeg;
		final String source;
		final String targetName;
		final double horizontalRangeCm;
		final boolean stale;

		Aim(double alphaDeg, String source, String targetName, double horizontalRangeCm, boolean stale) {
			this.alphaDeg = alphaDeg;
			this.source = source;
			this.targetName = targetName;
			this.horizontalRangeCm = horizontalRangeCm;
			this.stale = stale;
		}
	}

	/**
	 * Vision target if a matching non-stale cluster is visible, else the localization fallback (§4.2).
	 * Per-run offsets are applied here: {@code α += aimOffsetDeg + deg(atan2(lateralOffsetCm,
	 * horizontalRangeCm))} — the horizontal distance is the geometrically correct denominator for a
	 * horizontal shift at the target (deviation D1), keeping both paths in the same units.
	 */
	private Aim computeAim(double angle) {
		double aimOffset = aimOffsetDeg;
		double lateralOffset = lateralOffsetCm;

		Double manual = manualGoal;
		if (manual != null) {
			// Bench/tuning override: an absolute aim angle. No target range exists here, so the
			// lateral offset has no denominator and only the angular offset applies.
			return new Aim(
					manual + aimOffset, SOURCE_MANUAL, String.format(Locale.US, "MANUAL(%.1f)", manual), -1.0, false);
		}

		ClusterTarget best = pickVisionTarget();
		if (best != null) {
			// β = ftcPose.bearing is CCW-positive (positive = target left of the camera's forward
			// axis, verified against RobotAutoDriveToAprilTagOmni), so α = θ + mountYaw + β.
			double alpha = angle + TurretConfig.cameraMountYawDeg + best.rawBearing;
			double rangeCm = best.resolved.horizontalDistance * 2.54;
			return new Aim(
					alpha + aimOffset + lateralShift(lateralOffset, rangeCm),
					SOURCE_VISION,
					best.name,
					rangeCm,
					best.stale);
		}

		double[] cell = cellPosition();
		if (cell != null && localization != null) {
			Pose pose = localization.getPose();
			if (pose != null) {
				double dx = cell[0] - pose.x();
				double dy = cell[1] - pose.y();
				double alpha = wrapDeg(Math.toDegrees(Math.atan2(dy, dx)) - Math.toDegrees(pose.heading()));
				double rangeCm = Math.hypot(dx, dy) * 2.54;
				return new Aim(
						alpha + aimOffset + lateralShift(lateralOffset, rangeCm),
						SOURCE_LOCALIZATION,
						String.format(Locale.US, "CELL(%.1f, %.1f)", cell[0], cell[1]),
						rangeCm,
						false);
			}
		}
		return null;
	}

	/** Positive lateral offset shifts the aim point left (CCW) at the target's range. */
	private static double lateralShift(double lateralOffsetCm, double horizontalRangeCm) {
		if (horizontalRangeCm <= 1e-6) {
			return 0.0;
		}
		return Math.toDegrees(Math.atan2(lateralOffsetCm, horizontalRangeCm));
	}

	/** Best (nearest) non-stale cluster for our alliance and the half of the field we are on. */
	private ClusterTarget pickVisionTarget() {
		if (vision == null) {
			return null;
		}
		List<ClusterTarget> targets = vision.getTargets();
		if (targets == null) {
			return null;
		}
		char letter = TurretConfig.allianceLetter();
		String half = halfKeyword();
		ClusterTarget best = null;
		for (ClusterTarget target : targets) {
			if (target == null || target.stale || target.name == null || target.name.isEmpty()) {
				continue;
			}
			if (Character.toUpperCase(target.name.charAt(0)) != letter) {
				continue;
			}
			if (half != null && !target.name.toUpperCase(Locale.US).contains(half)) {
				continue;
			}
			if (best == null || target.resolved.range < best.resolved.range) {
				best = target;
			}
		}
		return best;
	}

	/** "AUDIENCE" / "SCORING" for the half of the field the robot is physically on; null if unknown. */
	private String halfKeyword() {
		if (localization == null) {
			return null;
		}
		Pose pose = localization.getPose();
		if (pose == null) {
			return null;
		}
		return pose.y() > TurretConfig.halfSplitY ? "SCORING" : "AUDIENCE";
	}

	/** Fixed field position of the HIVE CELL for our alliance + half (PedroPathing inches). */
	private double[] cellPosition() {
		String half = halfKeyword();
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
	 * §4.3: among {@code α + 360k} inside [minAngle, maxAngle], pick the one closest to the current
	 * position — "go to 360+" vs "turn all the way around to near 0". The chosen goal is then a plain
	 * bounded linear position problem (continuous-input wrapping stays disabled in the controller).
	 */
	private static double selectGoal(double alpha, double currentAngle) {
		double min = TurretConfig.minAngleDeg;
		double max = TurretConfig.maxAngleDeg;
		double best = clamp(alpha, min, max);
		double bestDistance = Double.POSITIVE_INFINITY;
		for (int k = -1; k <= 1; k++) {
			double candidate = alpha + 360.0 * k;
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

	// ── gamepad (per-run offsets + always-face toggle) ─────────────────────────────

	// §4.7: dpad_left/right step aimOffsetDeg −/+ (left −, right +); dpad_up/down step
	// lateralOffsetCm +/− (up +, down −); hold left_bumper for 0.1 steps; y zeroes both.
	// The handlers exist for both gamepad prefixes; only the configured prefix acts.

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
			zeroOffsets();
		}
	}

	@SubscribedTo(topic = "g2/y/rising")
	public void g2ZeroOffsets() {
		if ("g2".equals(inputPrefix)) {
			zeroOffsets();
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

	private void zeroOffsets() {
		aimOffsetDeg = 0.0;
		lateralOffsetCm = 0.0;
	}

	private double stepSize() {
		Boolean bumper = orchestrator
				.getLatestValue(inputPrefix + "/left_bumper", Boolean.class)
				.orElse(false);
		return bumper != null && bumper ? 0.1 : 1.0;
	}

	// ── helpers ───────────────────────────────────────────────────────────────────

	/** Refresh gains/constraints from the Panels statics (read live every tick, §4.6). */
	private void refreshController() {
		profiledPid.setPID(nonNegative(TurretConfig.kp), nonNegative(TurretConfig.ki), nonNegative(TurretConfig.kd));
		profiledPid.setIZone(Math.max(TurretConfig.iZone, 0.0));
		profiledPid.setIntegratorRange(TurretConfig.integratorMin, TurretConfig.integratorMax);
		profiledPid.setTolerance(Math.max(TurretConfig.positionToleranceDeg, 0.0));

		double v = Math.max(TurretConfig.maxVelocityDegPerSec, MIN_CONSTRAINT);
		double a = Math.max(TurretConfig.maxAccelerationDegPerSec2, MIN_CONSTRAINT);
		if (v != constraintsV || a != constraintsA) {
			profiledPid.setConstraints(new TrapezoidProfile.Constraints(v, a));
			constraintsV = v;
			constraintsA = a;
		}

		feedforward.setKs(TurretConfig.ks);
		feedforward.setKv(nonNegative(TurretConfig.kv));
		feedforward.setKa(nonNegative(TurretConfig.ka));
	}

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

	/** Tuning override: face this absolute aim angle (still goal-selected and profiled); null = cells. */
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

	/** Primary driver-station line (§4.9). Verbose detail goes below the block via the verbose pass. */
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

	/** Verbose detail (behind {@link TurretConfig#verboseTelemetry}), rendered below the primary block. */
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

	/** Sliding average of the most recent {@link #SETTLE_WINDOW_SAMPLES} |error| samples. */
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
