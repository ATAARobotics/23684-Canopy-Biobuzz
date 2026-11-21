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
	public static double KICKER_POWER = 0.8;
	public static double STOP_POWER = 0.0;

	// Set by auto (or any other caller) to drive the kicker directly. While non-null it wins over
	// the Shooter's target RPM, until clearPowerOverride() hands control back.
	private volatile Double powerOverride;

	// Last target RPM seen, so releasing an override can re-apply the RPM-driven power straight
	// away instead of waiting for the next Shooter tick.
	private volatile Double lastTargetRPM;

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
		lastTargetRPM = targetRPM;
		if (powerOverride != null) {
			return;
		}
		apply(powerFor(targetRPM));
	}

	private static double powerFor(Double targetRPM) {
		return targetRPM != null && targetRPM > Shooter.MIN_RPM ? KICKER_POWER : STOP_POWER;
	}

	private void apply(double power) {
		// Write raw so it lands on this tick: run() would re-queue through the gate
		// executor instead of gating the write inline.
		if (!orchestrator.hardware().isRestricted("write")) {
			shooterKicker.raw().setPower(power);
		}
		orchestrator.publish("shooterKicker/power", power);
	}

	/**
	 * Drive the kicker from auto: {@code power} is applied now and held until
	 * {@link #clearPowerOverride()} is called, ignoring the Shooter's target RPM. Pass {@code null}
	 * to release the override.
	 */
	public void setPowerOverride(Double power) {
		powerOverride = power;
		if (power != null) {
			apply(power);
		} else {
			apply(powerFor(lastTargetRPM));
		}
	}

	/** Release a {@link #setPowerOverride(Double)} so the Shooter's RPM drives the kicker again. */
	public void clearPowerOverride() {
		setPowerOverride(null);
	}

	/** Whether auto currently owns the kicker. */
	public boolean isOverridden() {
		return powerOverride != null;
	}
}
