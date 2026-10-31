package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.control.controller.PIDController;
import com.aaravlabs.control.controller.SimpleMotorFeedforward;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class Shooter extends Node {

	private final SafeDevice<DcMotorEx> shooter;
	public static double kP = 0.001, kI = 0, kD = 0;
	public static double kV = 0.00063, kS = 0.17;
	PIDController shooterPID;
	SimpleMotorFeedforward shooterFF;
	double targetRPM = 0.0;
	double currentRPM = 0.0;
	public static double STOP_POWER = 0.0;
	public boolean atRPM;

	public Shooter(Orchestrator orchestrator, SafeDevice<DcMotorEx> shooter) {
		super(orchestrator);
		this.shooter = shooter;
		shooterFF = new SimpleMotorFeedforward(kS, kV, 0);
		shooterPID = new PIDController(kP, kI, kD);
		shooter.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
		orchestrator.getOrCreateTopic("shooter/currentRPM", Double.class);
		orchestrator.getOrCreateTopic("shooter/targetRPM", Double.class);
		orchestrator.getOrCreateTopic("shooter/atRPM", Boolean.class);
		orchestrator.getOrCreateTopic("shooter/RPM", Double.class);
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void update() {
		double targetRPM =
				orchestrator.getLatestValue("shooter/RPM", Integer.class).orElse(0);
		setTargetRPM(targetRPM);
		updateRPM();
		updateMotor();
		atRPM = (this.targetRPM > 80) && (Math.abs(this.targetRPM - currentRPM) < 50);

		orchestrator.publish("shooter/currentRPM", currentRPM);
		orchestrator.publish("shooter/targetRPM", this.targetRPM);
		orchestrator.publish("shooter/atRPM", atRPM);
	}

	private void updateRPM() {
		shooter.run(m -> {
			double velocity = m.getVelocity(AngleUnit.DEGREES); // degrees per second
			// Conversion: (deg/s) / 360 = rev/s. rev/s * 60 = RPM
			currentRPM = velocity / 6;
		});
	}

	private void updateMotor() {
		if (targetRPM < 80) {
			shooter.run(m -> m.setPower(STOP_POWER));
			shooterPID.reset();
		} else {
			double pidOutput = shooterPID.calculate(currentRPM, targetRPM);
			double ffOutput = shooterFF.calculate(targetRPM, 0);
			shooter.run(m -> m.setPower(pidOutput + ffOutput));
		}
	}

	public void setTargetRPM(double targetRPM) {
		this.targetRPM = targetRPM;
	}
}
