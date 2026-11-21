package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.OnHardwareThread;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class ShooterKicker extends Node {
	private final SafeDevice<DcMotorEx> shooterKicker;
	public static double KICKER_POWER = 1.0;
	public static double STOP_POWER = 0.0;

	public ShooterKicker(Orchestrator orchestrator, SafeDevice<DcMotorEx> shooterKicker) {
		super(orchestrator);
		this.shooterKicker = shooterKicker;
		shooterKicker.labels("write/shooterKicker").run(m -> {
			m.setDirection(DcMotor.Direction.FORWARD);
			m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		});

		orchestrator.getOrCreateTopic("shooterKicker/power", Double.class);
	}

	@OnHardwareThread
	@SubscribedTo(topic = "shooter/targetRPM")
	public void onTargetRPM(Double targetRPM) {
		double power = targetRPM != null && targetRPM > Shooter.MIN_RPM ? KICKER_POWER : STOP_POWER;
		// Write raw so it lands on this tick: run() would re-queue through the gate
		// executor instead of gating the write inline.
		if (!orchestrator.hardware().isRestricted("write")) {
			shooterKicker.raw().setPower(power);
		}
		orchestrator.publish("shooterKicker/power", power);
	}
}
