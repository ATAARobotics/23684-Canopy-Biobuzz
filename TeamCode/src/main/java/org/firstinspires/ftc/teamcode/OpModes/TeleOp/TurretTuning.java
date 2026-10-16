package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

/**
 * Bench tuning for the turret — hub + turret + encoder only (no vision, no localization).
 *
 * <p>Runs an automatic step cycle: a square-wave goal alternating {@code cycleBaseDeg} and {@code
 * cycleTargetDeg} every {@code cycleHalfPeriodSec} (all three live Panels Configurables in {@link
 * TurretConfig}). The goal goes through the same goal selection and profiled controller as match
 * code, so what you tune here is exactly what runs in MainTeleOp. Gains are edited live in Panels
 * while it cycles.
 *
 * <p>Gamepad (g1): the standard offset map, where positive means right. {@code dpad_left} and
 * {@code dpad_right} step the aim offset down and up, {@code dpad_up} and {@code dpad_down} step
 * the lateral offset up and down, 1.0 at a time or 0.1 while {@code left_bumper} is held, and
 * {@code y} zeroes both. {@code x} turns always-face off and on, {@code a} pauses the cycle with
 * the turret held at base, and {@code b} jumps straight to base.
 *
 * <p><b>Tuning order:</b> {@code kv} (smooth sweep), then {@code ks} (break stiction and hold at
 * the goal), then {@code ka} (only if acceleration tracking is poor), then {@code kp} (trim the
 * residual error), {@code kd} only if it oscillates, and {@code ki} last and small. Every gain
 * starts at zero, so the turret will not move at all until you give it something; the gain echo
 * line on screen shows what is loaded.
 *
 * <p>Bench checks before closing the loop: turn the encoder shaft one full turn by hand and
 * confirm the counts line changes by about 2048 (one turret turn is about 5120). Then check the two
 * signs independently: positive power should rotate the turret clockwise towards +angle
 * ({@code powerSign}), and the angle should increase as the turret turns clockwise
 * ({@code encoderReversed}). Get either wrong and the loop runs away instead of settling.
 */
@Configurable
@TeleOp(name = "Turret Tuning", group = "Tests")
public class TurretTuning extends SafeOpMode {

	private static final long RENDER_PERIOD_NANOS = 100_000_000L; // ~10 Hz screen updates

	private Turret turret;
	private TuningCycle cycle;
	private long lastRenderNanos;

	@Override
	protected void onSafeInit() {
		GamepadAdaptor.attach(orchestrator, gamepad1, "g1");

		turret = new Turret(
				orchestrator,
				safeMap,
				safeMap.device(CRServo.class, "turretServo"),
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

		telemetry.addData(
				"Turret tuning",
				"cycle  base %.0f°  target %.0f°  half-period %.1fs  [%s]",
				TurretConfig.cycleBaseDeg,
				TurretConfig.cycleTargetDeg,
				TurretConfig.cycleHalfPeriodSec,
				cycle.isRunning()
						? "RUNNING"
						: "PAUSED");
		telemetry.addData(
				"  track",
				"goal %.1f°   setpoint %.1f° @ %.0f°/s   angle %.1f°   err %+.1f°",
				turret.getGoalDeg(),
				turret.getSetpointDeg(),
				turret.getSetpointVelocityDegPerSec(),
				turret.getAngleDeg(),
				turret.getErrorDeg());
		telemetry.addData(
				"  volts",
				"ff %+.2fV   pid %+.2fV   total %+.2fV   battery %.1fV   power %+.2f",
				turret.getFfVolts(),
				turret.getPidVolts(),
				turret.getFfVolts() + turret.getPidVolts(),
				turret.getBatteryVoltage(),
				turret.getPower());
		telemetry.addData(
				"  encoder",
				"%d counts   %.4f counts/deg (%.0f × %.0f/%.0f ÷ 360)   %.4f°/count",
				turret.getEncoderTicks(),
				TurretConfig.countsPerTurretDegree(),
				TurretConfig.encoderCountsPerRev,
				TurretConfig.turretGearTeeth,
				TurretConfig.encoderGearTeeth,
				TurretConfig.degreesPerCount());
		telemetry.addData(
				"  last cycle",
				"peak err %.1f°  settle err %.1f°",
				turret.getLastCyclePeakErrorDeg(),
				turret.getLastCycleSettleErrorDeg());
		telemetry.addData(
				"  offsets", "aim %+.1f°  lateral %+.1f cm", turret.getAimOffsetDeg(), turret.getLateralOffsetCm());
		telemetry.addData(
				"  gains",
				"ks %.2f kv %.3f ka %.3f | kp %.2f ki %.3f kd %.3f",
				TurretConfig.ks,
				TurretConfig.kv,
				TurretConfig.ka,
				TurretConfig.kp,
				TurretConfig.ki,
				TurretConfig.kd);
		telemetry.update();
	}

	/** Drives the square-wave goal; pauses hold the turret at base ({@code a}, {@code b}). */
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
			// Cycle parameters are read live every tick so Panels edits take effect mid-run.
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
