package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends Node {
	private final SafeDevice<DcMotorEx> intake;
	private final SafeDevice.Scope<DcMotorEx> intakeSet;

	public Intake(Orchestrator orchestrator, SafeDevice<DcMotorEx> intake) {
		super(orchestrator);
		this.intake = intake;
		this.intakeSet = intake.labels("write/intake").slot("intake/power");
		intake.labels("write/intake").run(m -> m.setDirection(DcMotorSimple.Direction.REVERSE));
	}

	@SubscribedTo(topic = "g2/left_stick_y")
	public void onManualPower(Float stickY) {
		double applied = stickY != null ? -stickY.doubleValue() : 0.0;
		intakeSet.run(m -> m.setPower(applied));
		orchestrator.publish("intake/power", applied);
	}

	@Override
	public void close() {
		intakeSet.run(m -> m.setPower(0.0));
	}
}
