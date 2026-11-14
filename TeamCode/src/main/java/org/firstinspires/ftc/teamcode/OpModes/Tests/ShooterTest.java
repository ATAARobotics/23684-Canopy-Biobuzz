package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Subsystems.HoodServo;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterKicker;

@Configurable
@TeleOp(name = "Shooter Test", group = "Tests")
public class ShooterTest extends SafeOpMode {

	private SafeDevice<DcMotorEx> shooter;
	private SafeDevice<DcMotorEx> shooterKicker;
	private SafeDevice<Servo> hoodServo;

	JoinedTelemetry joinedTelemetry;

	@Override
	protected void onSafeInit() {
		GamepadAdaptor.attach(orchestrator, gamepad2, "g2");
		shooter = safeMap.device(DcMotorEx.class, "pollenShooter");
		orchestrator.registerNode("shooter", new Shooter(orchestrator, shooter, "g2"));

		shooterKicker = safeMap.device(DcMotorEx.class, "shooterKicker");
		orchestrator.registerNode("shooterKicker", new ShooterKicker(orchestrator, shooterKicker));

		hoodServo = safeMap.device(Servo.class, "hoodServo");
		orchestrator.registerNode("hoodServo", new HoodServo(orchestrator, hoodServo));

		joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
	}

	@Override
	protected void onSafeLoop() {
		joinedTelemetry.addData(
				"Target",
				orchestrator.getLatestValue("shooter/targetRPM", Double.class).orElse(0.0));
		joinedTelemetry.addData(
				"Current",
				orchestrator.getLatestValue("shooter/currentRPM", Double.class).orElse(0.0));
		joinedTelemetry.addData(
				"At RPM",
				orchestrator.getLatestValue("shooter/atRPM", Boolean.class).orElse(false));
		joinedTelemetry.addData(
				"Kicker Power",
				orchestrator.getLatestValue("shooterKicker/power", Double.class).orElse(0.0));
		joinedTelemetry.addData(
				"Hood Position",
				orchestrator
						.getLatestValue("hood/targetPosition", Boolean.class)
						.orElse(false));
		joinedTelemetry.update();
	}
}
