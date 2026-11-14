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
public class Shooter extends Node {
	private final SafeDevice<DcMotorEx> shooter;
	public static double kP = 0.00012, kI = 0, kD = 0;
	public static double kV = 0.000173, kS = 0.04;
	PIDController shooterPID;
	SimpleMotorFeedforward shooterFF;
	private volatile double targetRPM = 0.0;
	private volatile double currentRPM = 0.0;
	public static double STOP_POWER = 0.0;
	public static int MIN_RPM = 80;
	public static int RPM_TOLERANCE = 50;
	public boolean atRPM;
	private final String inputPrefix;
	public static double TICKS_PER_REVOLUTION = 28.0;
	public static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
	public static double RPM_TARGET = 4000.0;

	public Shooter(Orchestrator orchestrator, SafeDevice<DcMotorEx> shooter, String inputPrefix) {
		super(orchestrator);
		this.shooter = shooter;
		shooterFF = new SimpleMotorFeedforward(kS, kV, 0);
		shooterPID = new PIDController(kP, kI, kD);
		this.inputPrefix = inputPrefix;
		shooter.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
		orchestrator.getOrCreateTopic("shooter/currentRPM", Double.class);
		orchestrator.getOrCreateTopic("shooter/targetRPM", Double.class);
		orchestrator.getOrCreateTopic("shooter/atRPM", Boolean.class);
		orchestrator.publish("shooter/targetRPM", 0.0);
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void update() {
		refreshController();

		targetRPM =
				orchestrator.getLatestValue("shooter/targetRPM", Double.class).orElse(0.0);
		currentRPM = shooter.raw().getVelocity() * RPM_CONVERSION; // rotations per minute
		atRPM = (targetRPM > MIN_RPM) && (Math.abs(targetRPM - currentRPM) < RPM_TOLERANCE);

		if (targetRPM > MIN_RPM) {
			double pidOutput = shooterPID.calculate(currentRPM, targetRPM);
			double ffOutput = shooterFF.calculate(targetRPM);
			shooter.raw().setPower(pidOutput + ffOutput);
		} else {
			shooter.raw().setPower(STOP_POWER);
			shooterPID.reset();
		}

		orchestrator.publish("shooter/currentRPM", currentRPM);
		orchestrator.publish("shooter/atRPM", atRPM);
	}

	private void refreshController() {
		shooterPID.setPID(kP, kI, kD);
		shooterFF.setKs(kS);
		shooterFF.setKv(kV);
	}

	@SubscribedTo(topic = "g2/right_bumper/rising")
	public void g2RightBumperRising() {
		if ("g2".equals(inputPrefix)) {
			orchestrator.publish("shooter/targetRPM", RPM_TARGET);
		}
	}

	@SubscribedTo(topic = "g2/right_bumper/falling")
	public void g2RightBumperFalling() {
		if ("g2".equals(inputPrefix)) {
			orchestrator.publish("shooter/targetRPM", 0.0);
		}
	}
}
