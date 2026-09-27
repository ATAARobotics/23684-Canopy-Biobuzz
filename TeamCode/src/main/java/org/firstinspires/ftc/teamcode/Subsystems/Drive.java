package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drive extends Node {

	private final SafeDevice<DcMotorEx> fl;
	private final SafeDevice<DcMotorEx> fr;
	private final SafeDevice<DcMotorEx> bl;
	private final SafeDevice<DcMotorEx> br;

	public Drive(
			Orchestrator orchestrator,
			SafeDevice<DcMotorEx> frontLeft,
			SafeDevice<DcMotorEx> frontRight,
			SafeDevice<DcMotorEx> backLeft,
			SafeDevice<DcMotorEx> backRight) {
		super(orchestrator);
		this.fl = frontLeft;
		this.fr = frontRight;
		this.bl = backLeft;
		this.br = backRight;

		fl.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
		bl.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
		fr.run(m -> m.setDirection(DcMotor.Direction.REVERSE));
		br.run(m -> m.setDirection(DcMotor.Direction.REVERSE));
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void drive() {
		double y = -orchestrator.getLatestValue("g1/left_stick_y", Float.class).map(Float::doubleValue).orElse(0.0);
		double x = orchestrator.getLatestValue("g1/left_stick_x", Float.class).map(Float::doubleValue).orElse(0.0) * 1.1;
		double r = -orchestrator.getLatestValue("g1/right_stick_x", Float.class).map(Float::doubleValue).orElse(0.0);

		double pFL = y + x + r;
		double pFR = y - x - r;
		double pBL = y - x + r;
		double pBR = y + x - r;
		double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

		fl.run(m -> m.setPower(pFL / max));
		fr.run(m -> m.setPower(pFR / max));
		bl.run(m -> m.setPower(pBL / max));
		br.run(m -> m.setPower(pBR / max));

		orchestrator.publish("drive/power/fl", pFL / max);
		orchestrator.publish("drive/power/fr", pFR / max);
		orchestrator.publish("drive/power/bl", pBL / max);
		orchestrator.publish("drive/power/br", pBR / max);
	}
}