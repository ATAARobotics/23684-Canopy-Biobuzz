package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.Topic;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.Optional;

public class Drive extends Node {

	/**
	 * Topics an external controller (e.g. PollenPursuit) publishes to take over the
	 * drivetrain. All three must be fresh for the override to engage; if any is missing
	 * or stale the gamepad is used instead. Nothing publishes these during normal teleop,
	 * so MainTeleOp behaviour is unchanged.
	 */
	public static final String CMD_FORWARD = "drive/cmd/y";
	public static final String CMD_STRAFE = "drive/cmd/x";
	public static final String CMD_TURN = "drive/cmd/r";

	/** How long a published command stays authoritative before falling back to the gamepad. */
	private static final long CMD_TIMEOUT_NANOS = 250_000_000L;

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

	/** Latest value on a command topic, or null if absent or older than the override window. */
	private Double liveCommand(String topic) {
		Optional<Topic<Double>> t = orchestrator.findTopic(topic, Double.class);
		if (t.isEmpty()) {
			return null;
		}
		if (System.nanoTime() - t.get().latestPublishNanos() > CMD_TIMEOUT_NANOS) {
			return null;
		}
		return t.get().latestValueOr(null);
	}

	@RunPeriodically(hz = 50, hardware = true)
	public void drive() {
		double y;
		double x;
		double r;

		Double cmdY = liveCommand(CMD_FORWARD);
		Double cmdX = liveCommand(CMD_STRAFE);
		Double cmdR = liveCommand(CMD_TURN);

		if (cmdY != null && cmdX != null && cmdR != null) {
			y = cmdY;
			x = cmdX;
			r = cmdR;
		} else {
			y = -orchestrator
					.getLatestValue("g1/left_stick_y", Float.class)
					.map(Float::doubleValue)
					.orElse(0.0);
			x = orchestrator
					.getLatestValue("g1/left_stick_x", Float.class)
					.map(Float::doubleValue)
					.orElse(0.0) * 1.1;
			r = -orchestrator
					.getLatestValue("g1/right_stick_x", Float.class)
					.map(Float::doubleValue)
					.orElse(0.0);
		}

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
