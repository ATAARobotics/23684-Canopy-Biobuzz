package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.control.controller.PIDController;
import com.aaravlabs.control.controller.SimpleMotorFeedforward;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
public class  Shooter extends Node {
	private final SafeDevice<DcMotorEx> shooter;
	public static double kP = 0.00016, kI = 0.0, kD = 0.0;
	public static double kV = 0.000186, kS = 0.12, kA = 0.0;
	PIDController shooterPID;
	SimpleMotorFeedforward shooterFF;
	private volatile double targetRPM = 0.0;
	private volatile double currentRPM = 0.0;
	public static double STOP_POWER = 0.0;
	public static int MIN_RPM = 80;
	public static int RPM_TOLERANCE = 100;
	public boolean atRPM;
	private final String inputPrefix;
	private final boolean refreshControllers;
	public static double TICKS_PER_REVOLUTION = 28.0;
	public static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
	public static double RPM_TARGET = 3500.0;

	public Shooter(Orchestrator orchestrator, SafeDevice<DcMotorEx> shooter, String inputPrefix) {
		this(orchestrator, shooter, inputPrefix, false);
	}

	public Shooter(
			Orchestrator orchestrator, SafeDevice<DcMotorEx> shooter, String inputPrefix, boolean refreshControllers) {
		super(orchestrator);
		this.shooter = shooter;
		this.inputPrefix = inputPrefix;
		this.refreshControllers = refreshControllers;
		shooterFF = new SimpleMotorFeedforward(kS, kV, kA);
		shooterPID = new PIDController(kP, kI, kD);
		shooter.labels("write/shooter").run(m -> m.setDirection(DcMotor.Direction.REVERSE));
		orchestrator.getOrCreateTopic("shooter/currentRPM", Double.class);
		orchestrator.getOrCreateTopic("shooter/targetRPM", Double.class);
		orchestrator.getOrCreateTopic("shooter/atRPM", Boolean.class);
		orchestrator.publish("shooter/targetRPM", 0.0);
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void update() {
		if (refreshControllers) {
			refreshController();
		}

		targetRPM =
				orchestrator.getLatestValue("shooter/targetRPM", Double.class).orElse(0.0);
		currentRPM = shooter.raw().getVelocity() * RPM_CONVERSION; // rotations per minute
		// "At speed" is at or above the tolerance band, not just inside it: the wheel
		// overshoots on the way up and can still be above target when the setpoint drops.
		atRPM = (targetRPM > MIN_RPM) && (currentRPM >= targetRPM - RPM_TOLERANCE);

		// Same pattern as the Turret: this 50 Hz control loop runs on the hardware
		// thread, so it must not hand the write back to the gate executor, which would
		// put an unknown gap between the velocity read and the write. It checks the
		// restriction inline and writes the motor raw instead.
		boolean held = orchestrator.hardware().isRestricted("write");
		if (targetRPM > MIN_RPM) {
			double pidOutput = shooterPID.calculate(currentRPM, targetRPM);
			double ffOutput = shooterFF.calculate(targetRPM);
			if (!held) {
				shooter.raw().setPower(pidOutput + ffOutput);
			}
		} else {
			if (!held) {
				shooter.raw().setPower(STOP_POWER);
			}
			shooterPID.reset();
		}

		orchestrator.publish("shooter/currentRPM", currentRPM);
		orchestrator.publish("shooter/atRPM", atRPM);
	}

	private void refreshController() {
		shooterPID.setPID(kP, kI, kD);
		shooterFF.setKs(kS);
		shooterFF.setKv(kV);
		shooterFF.setKa(kA);
	}

	@SubscribedTo(topic = "g2/right_trigger_pressed/rising")
	public void g2RightBumperRising() {
		if ("g2".equals(inputPrefix)) {
			orchestrator.publish("shooter/targetRPM", RPM_TARGET);
		}
	}

	@SubscribedTo(topic = "g2/right_trigger_pressed/falling")
	public void g2RightBumperFalling() {
		if ("g2".equals(inputPrefix)) {
			orchestrator.publish("shooter/targetRPM", 0.0);
		}
	}
}
