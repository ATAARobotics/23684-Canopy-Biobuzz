package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake extends Node {

	private final SafeDevice<DcMotorEx> intake;

	public Intake(Orchestrator orch, SafeDevice<DcMotorEx> intake) {
		super(orch);
		this.intake = intake;
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void runIntake() {
		double power = -orchestrator.getLatestValue("g2/left_stick_y", Float.class).map(Float::doubleValue).orElse(0.0);
		intake.run(m -> m.setPower(power));
	}
}