package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.control.controller.PIDController;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.OnHardwareThread;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.FieldMap;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;

/**
 * Aims the turret at our alliance's HIVE cell from the cell's field position and the robot pose. A manual goal
 * overrides it. Angles grow clockwise from straight ahead and are not wrapped.
 */
public class Turret extends Node {

	public static final String SOURCE_LOCALIZATION = "LOCALIZATION";
	public static final String SOURCE_MANUAL = "MANUAL";
	public static final String SOURCE_OFF = "OFF";

	private static final double LOOP_PERIOD_SEC = 0.02;
	// Stop pushing once the turret is this close to a limit.
	private static final double LIMIT_EPS_DEG = 0.5;
	// 25 ticks is 0.5 s at 50 Hz. The voltage read is the slow part.
	private static final int BATTERY_REFRESH_TICKS = 25;
	// Number of |error| samples behind the settle reading on the tuning screen.
	private static final int SETTLE_WINDOW_SAMPLES = 25;
	// While the turret is disabled (x), the owning gamepad's left stick X spins it directly. Full stick
	// (1.0) commands this fraction of power; centred (0.0) commands none.
	private static final double MANUAL_TURN_SCALE = 0.5;
	// Left stick X magnitude below this counts as centred, so drift doesn't creep the turret while disabled.
	private static final double MANUAL_TURN_DEADZONE = 0.05;
	private static final double FINE_TRIM_DEADZONE = 0.05;
	// Only treat the turret as manually moved if it turned this many degrees while disabled.
	private static final double MANUAL_MOVE_DEADZONE_DEG = 2.0;

	private final SafeDevice<CRServo> turretServo1;
	private final SafeDevice<CRServo> turretServo2;
	private final SafeDevice<CRServo> turretServo3;
	private final SafeDevice<DcMotorEx> encoderMotor;
	private final SafeHardwareMap safeMap;
	// Null on the tuner: the tuning OpMode has no odometry.
	private final Follower follower;
	// Alliance this turret aims for: 'B' or 'R'. Comes from the running OpMode's Team (getTeam().allianceLetter()),
	// so BlueTeleOp aims at the blue cell instead of a stale config default.
	private final char allianceLetter;

	// Angle when the turret was disabled; compared on re-enable to detect manual movement.
	private volatile double disabledAngle;

	private final PIDController pid;

	// Driver-set during a run. Instance state, not Panels statics, which outlive a match.
	private volatile double aimOffsetDeg;
	private volatile double lateralOffsetCm;
	private volatile boolean enabled = TurretConfig.enabledByDefault;

	// Bench override: face this absolute angle instead of a cell.
	private volatile Double manualGoal;

	// Owning gamepad's left stick X (-1..1), read to spin the turret while it is disabled.
	private volatile double manualTurnInput;
	private volatile boolean fineTrim;

	// Whether the previous tick had a target. Re-acquiring one re-seeds the controller.
	private boolean hadTarget;

	private int lastRawTicks;
	private boolean hasRawTicks;
	private int unwrappedTicks;
	// Zero captured on the first tick, when the turret faces straight ahead at init.
	private boolean zeroCaptured;

	private int batteryTick;
	private double batteryVoltage = TurretConfig.nominalVoltage;

	// Latched when the goal changes, so the tuning screen reports the cycle that just finished.
	private final SettleWindow settleWindow = new SettleWindow();
	private boolean goalInitialized;
	private double lastGoal;
	private double cyclePeakErrorDeg;
	private volatile double lastCyclePeakErrorDeg;
	private volatile double lastCycleSettleErrorDeg;

	// Loop state snapshots published for telemetry and read by the getters.
	private volatile double angleDeg;
	private volatile double aimDeg;
	private volatile double goalDeg;
	private volatile double errorDeg;
	private volatile double ffVolts;
	private volatile double pidVolts;
	private volatile double power;
	private volatile boolean atLimit;
	private volatile int encoderTicks;
	private volatile String source = SOURCE_OFF;
	private volatile String targetName = "—";

	public Turret(
			Orchestrator orchestrator,
			SafeHardwareMap safeMap,
			SafeDevice<CRServo> turretServo1,
			SafeDevice<CRServo> turretServo2,
			SafeDevice<CRServo> turretServo3,
			SafeDevice<DcMotorEx> encoderMotor,
			Follower follower,
			char allianceLetter) {
		super(orchestrator);
		this.safeMap = safeMap;
		this.turretServo1 = turretServo1;
		this.turretServo2 = turretServo2;
		this.turretServo3 = turretServo3;
		this.encoderMotor = encoderMotor;
		this.follower = follower;
		this.allianceLetter = allianceLetter;

		this.pid = new PIDController(TurretConfig.kp, TurretConfig.ki, TurretConfig.kd, LOOP_PERIOD_SEC);

		// Set servo direction so positive power turns the turret clockwise, matching the
		// encoder sign convention. Replaces the old powerSign constant.
		turretServo1.raw().setDirection(DcMotor.Direction.REVERSE);
		turretServo2.raw().setDirection(DcMotor.Direction.REVERSE);
		turretServo3.raw().setDirection(DcMotor.Direction.REVERSE);
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void controlLoop() {
		double angle = measureAngle();
		refreshBattery();

		Aim aim = enabled ? computeAim() : null;
		boolean hasTarget = aim != null;

		double goal;
		double power;
		atLimit = false;

		if (enabled) {
			goal = hasTarget ? track(angle, aim) : holdPosition(angle);
			hadTarget = hasTarget;
			if (hasTarget) {
				recordCycle(goal);
			}

			double commandedVolts = ffVolts + pidVolts;

			power = hasTarget ? toPower(commandedVolts) : 0.0;

			// Test the commanded volts, not the servo power, so the limit check is independent of
			// the servo direction set in the constructor.
			if (angle >= TurretConfig.maxAngleDeg - LIMIT_EPS_DEG && commandedVolts > 0.0) {
				power = 0.0;
				atLimit = true;
			} else if (angle <= TurretConfig.minAngleDeg + LIMIT_EPS_DEG && commandedVolts < 0.0) {
				power = 0.0;
				atLimit = true;
			}
		} else {
			// Disabled: the left stick X spins the turret directly (full stick → MANUAL_TURN_SCALE).
			// Still honour the travel limits.
			goal = angle;
			hadTarget = false;
			holdPosition(angle);
			power = MANUAL_TURN_SCALE * manualTurnInput;
			double angleDirection = power;
			if (angle >= TurretConfig.maxAngleDeg - LIMIT_EPS_DEG && angleDirection > 0.0) {
				power = 0.0;
				atLimit = true;
			} else if (angle <= TurretConfig.minAngleDeg + LIMIT_EPS_DEG && angleDirection < 0.0) {
				power = 0.0;
				atLimit = true;
			}
		}

		boolean held = orchestrator.hardware().isRestricted("write");

		if (!held) {
			turretServo1.raw().setPower(power);
			turretServo2.raw().setPower(power);
			turretServo3.raw().setPower(power);
		}

		publish(angle, goal, aim, power);
	}

	private double measureAngle() {
		int raw = encoderMotor.raw().getCurrentPosition();
		unwrappedTicks = unwrap(raw);
		// Capture the zero on the first tick, when the turret faces straight ahead at init. Without this the
		// fixed encoderZeroTicks (default 0) leaves the aim offset by however far the encoder reads from 0 at
		// power-on. This is the same auto-zero the staging turret used.
		if (!zeroCaptured) {
			TurretConfig.encoderZeroTicks = unwrappedTicks;
			zeroCaptured = true;
		}
		encoderTicks = unwrappedTicks - (int) Math.round(TurretConfig.encoderZeroTicks);
		// encoderReversed flips the sign so a positive angle is clockwise, matching the servo direction set in
		// the constructor and the staging config. Wrong sign here runs the turret away from the target.
		double sign = TurretConfig.encoderReversed ? -1.0 : 1.0;
		return sign * encoderTicks / TurretConfig.countsPerTurretDegree();
	}

	private int unwrap(int raw) {
		if (!hasRawTicks) {
			hasRawTicks = true;
			lastRawTicks = raw;
			unwrappedTicks = raw;
			return unwrappedTicks;
		}
		int fullScale = (int) Math.round(360.0 * TurretConfig.countsPerTurretDegree());
		int delta = raw - lastRawTicks;
		if (fullScale > 0) {
			if (delta > fullScale / 2) {
				delta -= fullScale;
			} else if (delta < -fullScale / 2) {
				delta += fullScale;
			}
		}
		lastRawTicks = raw;
		unwrappedTicks += delta;
		return unwrappedTicks;
	}

	// Nothing to aim at: hold still and re-seed the controller. Without this, the next target would start
	// from a stale setpoint.
	private double holdPosition(double angle) {
		pid.reset();
		ffVolts = 0.0;
		pidVolts = 0.0;
		errorDeg = 0.0;
		return angle;
	}

	// Pick the goal inside the travel range, then advance the PID by one tick.
	// Re-seed from the measurement on the first tick and whenever a target is re-acquired.
	private double track(double angle, Aim aim) {
		double goal = selectGoal(aim.angleDeg, angle);
		if (!goalInitialized || !hadTarget) {
			pid.reset();
		}
		refreshController();
		pidVolts = pid.calculate(angle, goal);
		// Static friction feedforward: ks * sign(error), inlining SimpleMotorFeedforward's
		// ks term without the velocity/acceleration terms that need a motion profile.
		double error = goal - angle;
		ffVolts = Math.abs(error) > 1e-9 ? TurretConfig.ks * Math.signum(error) : 0.0;
		errorDeg = error;
		return goal;
	}

	private double toPower(double volts) {
		return clamp(volts / Math.max(batteryVoltage, 1e-6), -1.0, 1.0);
	}

	private void refreshBattery() {
		batteryTick--;
		if (batteryTick <= 0) {
			batteryTick = BATTERY_REFRESH_TICKS;
			batteryVoltage = readBatteryVoltage();
		}
	}

	// Keep the peak and settle error of this cycle until the goal moves on.
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
	}

	// ── aiming ────────────────────────────────────────────────────────────────────

	/** What the turret should be facing this tick. */
	private static final class Aim {
		// Absolute turret angle in degrees clockwise from the robot's forward, the same frame as the measured
		// angle. Adding the current angle here would make the goal chase the turret.
		final double angleDeg;
		final String source;
		final String targetName;

		Aim(double angleDeg, String source, String targetName) {
			this.angleDeg = angleDeg;
			this.source = source;
			this.targetName = targetName;
		}
	}

	// Absolute turret angle to aim at, or null when there is nothing to aim at.
	private Aim computeAim() {
		Double manual = manualGoal;
		if (manual != null) {
			// The bench override is an absolute angle, so the lateral offset has no range to scale against.
			// Only the aim offset applies.
			return new Aim(manual + aimOffsetDeg, SOURCE_MANUAL, String.format(Locale.US, "MANUAL(%.1f)", manual));
		}

		Pose pose = follower == null ? null : follower.pose();
		if (pose == null) {
			return null;
		}
		String half = fieldHalf(pose);
		if (half == null) {
			return null;
		}

		FieldMap.Cluster cell = FieldMap.forCell(allianceLetter == 'B', "SCORING".equals(half));
		Pose pivot = pose.compose(new Pose(TurretConfig.offsetX, TurretConfig.offsetY, 0.0));

		double dx = cell.x - pivot.x();
		double dy = cell.y - pivot.y();
		// Pedro's heading grows counter-clockwise and ours clockwise, so heading − cell azimuth gives the
		// absolute turret angle. The current angle must not be added.
		double bearing = Math.toDegrees(pose.heading() - Math.atan2(dy, dx));
		double rangeCm = Math.hypot(dx, dy) * 2.54;
		return new Aim(
				bearing + aimOffsetDeg + lateralShift(rangeCm),
				SOURCE_LOCALIZATION,
				String.format(Locale.US, "CELL(%.1f, %.1f)", cell.x, cell.y));
	}

	// Extra angle for a shooter that sits off the turret axis: the lateral offset seen from the target's range.
	// Positive moves the aim right of the target.
	private double lateralShift(double rangeCm) {
		if (rangeCm <= 1e-6) {
			return 0.0;
		}
		return Math.toDegrees(Math.atan2(lateralOffsetCm, rangeCm));
	}

	// "SCORING" or "AUDIENCE" for the half of the field the robot is on, or null if unknown.
	private String fieldHalf(Pose pose) {
		if (pose == null) {
			return null;
		}
		return pose.y() > TurretConfig.halfSplitY ? "SCORING" : "AUDIENCE";
	}

	private String fieldHalf() {
		return follower == null ? null : fieldHalf(follower.pose());
	}

	// Of the wanted angle and every whole turn of it, pick the copy inside the travel range closest to the
	// current angle, so a small correction is not sent the long way round. If no copy fits (the travel range
	// is narrower than 360°), this clamps to the nearest end, so the turret gets as close to the target as
	// the mechanics allow.
	private static double selectGoal(double angleWanted, double currentAngle) {
		double min = TurretConfig.minAngleDeg;
		double max = TurretConfig.maxAngleDeg;
		double best = clamp(angleWanted, min, max);
		double bestDistance = Math.abs(best - currentAngle);
		int firstTurn = (int) Math.floor((min - angleWanted) / 360.0);
		int lastTurn = (int) Math.ceil((max - angleWanted) / 360.0);
		for (int k = firstTurn; k <= lastTurn; k++) {
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

	@OnHardwareThread
	@SubscribedTo(topic = "g2/dpad_left/rising")
	public void dpadLeft() {
		nudgeAim(-1.0);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/dpad_right/rising")
	public void dpadRight() {
		nudgeAim(1.0);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/dpad_up/rising")
	public void dpadUp() {
		nudgeLateral(1.0);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/dpad_down/rising")
	public void dpadDown() {
		nudgeLateral(-1.0);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/y/rising")
	public void zeroOffsets() {
		aimOffsetDeg = 0.0;
		lateralOffsetCm = 0.0;
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/x/rising")
	public void toggleEnabled() {
		if (enabled) {
			disabledAngle = angleDeg;
			enabled = false;
		} else {
			reenable();
		}
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/b/rising")
	public void reenableFromB() {
		if (!enabled) {
			reenable();
		}
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/left_stick_x")
	public void leftStickX(Float stickX) {
		double raw = stickX != null ? stickX : 0.0;
		// Deadzone: ignore tiny deflections so stick drift doesn't spin the disabled turret.
		manualTurnInput = Math.abs(raw) < MANUAL_TURN_DEADZONE ? 0.0 : raw;
	}

	@OnHardwareThread
	@SubscribedTo(topic = "g2/right_stick_x")
	public void rightStickX(Float stickX) {
		double raw = stickX != null ? stickX : 0.0;
		fineTrim = Math.abs(raw) >= FINE_TRIM_DEADZONE;
	}

	// Re-enable. If the turret was turned by hand past the deadzone, shift the aim so it holds where it was
	// left instead of snapping back. Use the target, not the disable-time angle: the robot keeps moving while
	// disabled, so the target drifts away from where the turret was parked.
	private void reenable() {
		double moved = angleDeg - disabledAngle;
		if (Math.abs(moved) > MANUAL_MOVE_DEADZONE_DEG) {
			aimOffsetDeg += angleDeg - aimDeg;
		}
		enabled = true;
	}

	private double trimStep() {
		return fineTrim ? 0.1 : 1.0;
	}

	private void nudgeAim(double direction) {
		aimOffsetDeg += direction * trimStep();
	}

	private void nudgeLateral(double direction) {
		lateralOffsetCm += direction * trimStep();
	}

	// ── helpers ───────────────────────────────────────────────────────────────────

	// Push the live Panels gains and limits into the controller so edits land immediately.
	private void refreshController() {
		pid.setPID(nonNegative(TurretConfig.kp), nonNegative(TurretConfig.ki), nonNegative(TurretConfig.kd));
		pid.setIZone(Math.max(TurretConfig.iZone, 0.0));
		pid.setIntegratorRange(TurretConfig.integratorMin, TurretConfig.integratorMax);
		pid.setTolerance(Math.max(TurretConfig.positionToleranceDeg, 0.0));
	}

	// First positive reading on the bus, or the configured nominal voltage if none are.
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

	// ── tuning & telemetry ────────────────────────────────────────────────────

	/** Bench override: face this absolute angle instead of a cell. Null returns to the cells. */
	public void setManualGoal(Double goal) {
		manualGoal = goal;
	}

	/** Capture the current position as the zero (straight ahead). Also re-arms the first-tick auto-zero. */
	public void captureZero() {
		TurretConfig.encoderZeroTicks = unwrappedTicks;
		zeroCaptured = true;
		orchestrator.log("turret: encoderZeroTicks captured at " + (int) TurretConfig.encoderZeroTicks);
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

	/** Primary driver-station line. The extra detail goes in publishVerboseTelemetry. */
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
		telemetry.addData("Field half", fieldHalf());
	}

	/** Extra detail, only shown when TurretConfig.verboseTelemetry is on. */
	public void publishVerboseTelemetry(Telemetry telemetry) {
		telemetry.addData(
				"Turret ctrl",
				"ff %+.2fV  pid %+.2fV  total %+.2fV  battery %.1fV%s",
				ffVolts,
				pidVolts,
				ffVolts + pidVolts,
				batteryVoltage,
				atLimit
						? "  AT LIMIT"
						: "");
		// TEMP: manual-turn sign check; remove once the left-stick X direction is confirmed.
		telemetry.addData(
				"  manual",
				"stickX %+.2f  power %+.2f  angle %+.1f°",
				orchestrator.getLatestValue("g2/left_stick_x", Float.class).orElse(0f),
				power,
				angleDeg);
		if (follower != null) {
			Pose pose = follower.pose();
			if (pose != null) {
				telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
			}
		}
	}

	@Override
	public void close() {
		turretServo1.labels("write/turret").run(m -> m.setPower(0.0));
		turretServo2.labels("write/turret").run(m -> m.setPower(0.0));
		turretServo3.labels("write/turret").run(m -> m.setPower(0.0));
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
