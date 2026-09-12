package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Drive extends Node {

	private final SafeDevice<DcMotorEx> fl;
	private final SafeDevice<DcMotorEx> fr;
	private final SafeDevice<DcMotorEx> bl;
	private final SafeDevice<DcMotorEx> br;

	public Drive(
			Orchestrator orch,
			SafeDevice<DcMotorEx> frontLeft,
			SafeDevice<DcMotorEx> frontRight,
			SafeDevice<DcMotorEx> backLeft,
			SafeDevice<DcMotorEx> backRight) {
		super(orch);
		this.fl = frontLeft;
		this.fr = frontRight;
		this.bl = backLeft;
		this.br = backRight;

		fl.run(m -> m.setDirection(DcMotor.Direction.REVERSE));
		bl.run(m -> m.setDirection(DcMotor.Direction.REVERSE));
		fr.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
		br.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void drive() {
		double y = -orchestrator.getLatestValue("g1/left_stick_y", Float.class).map(Float::doubleValue).orElse(0.0);
		double x = orchestrator.getLatestValue("g1/left_stick_x", Float.class).map(Float::doubleValue).orElse(0.0);

		double pFL = y + x;
		double pFR = y - x;
		double pBL = y - x;
		double pBR = y + x;
		double max = Math.max(1.0, Math.max(Math.abs(pFL),
				Math.max(Math.abs(pFR), Math.max(Math.abs(pBL), Math.abs(pBR)))));

		fl.run(m -> m.setPower(pFL / max));
		fr.run(m -> m.setPower(pFR / max));
		bl.run(m -> m.setPower(pBL / max));
		br.run(m -> m.setPower(pBR / max));
	}
}