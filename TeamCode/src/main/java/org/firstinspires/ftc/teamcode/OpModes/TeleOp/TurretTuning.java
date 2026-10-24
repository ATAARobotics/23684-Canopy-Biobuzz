package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

/**
 * Turret tuning against a square-wave goal, with no vision or odometry. Every value is editable in Panels.
 * The goal steps between cycleBaseDeg and cycleTargetDeg every cycleHalfPeriodSec through the same controller
 * path as MainTeleOp.
 */
@Configurable
@TeleOp(name = "Turret Tuning", group = "Tests")
public class TurretTuning extends SafeOpMode {

	// Checks before running in TeleOp: full rotation of the turret to the limit should match the expected
	// limit, positive power must turn the turret clockwise (otherwise flip powerSign), and the angle should
	// increase clockwise (otherwise flip encoderReversed). These three are required for the shooter to
	// function in TeleOp.
	//
	// Gamepad 1: dpad left/right steps the aim offset in degrees and dpad up/down steps the lateral offset in cm,
	// 1.0 per press or 0.1 with left bumper held, y zeroes both, x toggles always-face, a pauses with the
	// turret at base, and b jumps to base.
	//
	// All gains at zero leave the turret still. The gain echo line shows what is loaded. Tune ks to break
	// static friction, kv for a smooth sweep, then ka only when movement during acceleration is poor. Use kp
	// to fix error, and kd only if it oscillates. Rarely touch ki.

	private static final long RENDER_PERIOD_NANOS = 100_000_000L; // ~10 Hz screen updates

	private Turret turret;
	private TuningCycle cycle;
	private JoinedTelemetry joinedTelemetry;
	private long lastRenderNanos;

	@Override
	protected void onSafeInit() {
		GamepadAdaptor.attach(orchestrator, gamepad1, "g1");

		joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		turret = new Turret(
				orchestrator,
				safeMap,
				safeMap.device(CRServo.class, "turret"),
				safeMap.device(DcMotorEx.class, "intake"),
				null,
				null,
				"g1");
		orchestrator.registerNode("turret", turret);

		cycle = new TuningCycle(orchestrator, turret);
		orchestrator.registerNode("tuningCycle", cycle);
	}

	@Override
	protected void onSafeLoop() {
		long now = System.nanoTime();
		if (now - lastRenderNanos < RENDER_PERIOD_NANOS) {
			return;
		}
		lastRenderNanos = now;

		joinedTelemetry.addData(
				"Turret tuning",
				String.format(
						Locale.US,
						"cycle  base %.0f°  target %.0f°  half-period %.1fs  [%s]",
						TurretConfig.cycleBaseDeg,
						TurretConfig.cycleTargetDeg,
						TurretConfig.cycleHalfPeriodSec,
						cycle.isRunning()
								? "RUNNING"
								: "PAUSED"));

		joinedTelemetry.addData("setpoint", turret.getSetpointDeg());
		joinedTelemetry.addData("current", turret.getAngleDeg());

		joinedTelemetry.addData(
				"  track",
				String.format(
						Locale.US,
						"goal %.1f°   setpoint %.1f° @ %.0f°/s   angle %.1f°   err %+.1f°",
						turret.getGoalDeg(),
						turret.getSetpointDeg(),
						turret.getSetpointVelocityDegPerSec(),
						turret.getAngleDeg(),
						turret.getErrorDeg()));
		joinedTelemetry.addData(
				"  volts",
				String.format(
						Locale.US,
						"ff %+.2fV   pid %+.2fV   total %+.2fV   battery %.1fV   power %+.2f",
						turret.getFfVolts(),
						turret.getPidVolts(),
						turret.getFfVolts() + turret.getPidVolts(),
						turret.getBatteryVoltage(),
						turret.getPower()));
		joinedTelemetry.addData(
				"  encoder",
				String.format(
						Locale.US,
						"%d counts   %.4f counts/deg (%.0f × %.0f/%.0f ÷ 360)   %.4f°/count",
						turret.getEncoderTicks(),
						TurretConfig.countsPerTurretDegree(),
						TurretConfig.encoderCountsPerRev,
						TurretConfig.turretGearTeeth,
						TurretConfig.encoderGearTeeth,
						TurretConfig.degreesPerCount()));
		joinedTelemetry.addData(
				"  last cycle",
				String.format(
						Locale.US,
						"peak err %.1f°  settle err %.1f°",
						turret.getLastCyclePeakErrorDeg(),
						turret.getLastCycleSettleErrorDeg()));
		joinedTelemetry.addData(
				"  offsets",
				String.format(
						Locale.US,
						"aim %+.1f°  lateral %+.1f cm",
						turret.getAimOffsetDeg(),
						turret.getLateralOffsetCm()));
		joinedTelemetry.addData(
				"  gains",
				String.format(
						Locale.US,
						"ks %.2f kv %.3f ka %.3f | kp %.2f ki %.3f kd %.3f",
						TurretConfig.ks,
						TurretConfig.kv,
						TurretConfig.ka,
						TurretConfig.kp,
						TurretConfig.ki,
						TurretConfig.kd));
		joinedTelemetry.update();
	}

	// Drives the square-wave goal; when paused (a/b), holds the turret at base.
	private static class TuningCycle extends Node {

		private final Turret turret;
		private volatile boolean running = true;
		private boolean atTarget;
		private long lastFlipNanos = System.nanoTime();

		TuningCycle(Orchestrator orchestrator, Turret turret) {
			super(orchestrator);
			this.turret = turret;
		}

		@RunPeriodically(hz = 50)
		public void tick() {
			double base = TurretConfig.cycleBaseDeg;
			if (!running) {
				atTarget = false;
				turret.setManualGoal(base);
				return;
			}
			double halfPeriod = Math.max(TurretConfig.cycleHalfPeriodSec, 0.05);
			double elapsedSec = (System.nanoTime() - lastFlipNanos) / 1e9;
			if (elapsedSec >= halfPeriod) {
				atTarget = !atTarget;
				lastFlipNanos = System.nanoTime();
			}
			// Read live each tick so Panels edits take effect mid-run.
			turret.setManualGoal(atTarget ? TurretConfig.cycleTargetDeg : base);
		}

		@SubscribedTo(topic = "g1/a/rising")
		public void toggleCycle() {
			running = !running;
			lastFlipNanos = System.nanoTime();
		}

		@SubscribedTo(topic = "g1/b/rising")
		public void baseNow() {
			running = false;
			atTarget = false;
			turret.setManualGoal(TurretConfig.cycleBaseDeg);
		}

		boolean isRunning() {
			return running;
		}
	}
}
