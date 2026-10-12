package org.firstinspires.ftc.teamcode.OpModes.Tests;

//import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.HoodServo;
import org.firstinspires.ftc.teamcode.Subsystems.Shoot;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Shooter Test OPMODE", group = "Testing")
public class ShooterTestOpmode extends SafeOpMode {

    private SafeDevice<DcMotorEx> shooter;
    private SafeDevice<Servo> hoodServo;
     public static int RPM = 1000;

     public static double servohoodangle = 0.7;

     JoinedTelemetry joinedTelemetry;

    @Override
    protected void onSafeInit() {
        GamepadAdaptor.attach(orchestrator, gamepad2, "g2");
        shooter = safeMap.device(DcMotorEx.class, "shooter");
        orchestrator.registerNode("shooter", new Shoot(orchestrator, shooter));

        hoodServo = safeMap.device(Servo.class, "hoodServo");
        orchestrator.registerNode("hoodServo", new HoodServo(orchestrator, hoodServo));

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    protected void onSafeLoop() {
        boolean isPressed = orchestrator.getLatestValue("g2/x", Boolean.class).orElse(false);

        if (isPressed) {
            orchestrator.publish("shooter/RPM", RPM); // Shoots at target RPM when button pressed
        } else {
            orchestrator.publish("shooter/RPM",  0);  // Stops shooter when button released
        }

        boolean angleHigher = orchestrator.getLatestValue("g2/a", Boolean.class).orElse(false);
        if (angleHigher) {
            orchestrator.publish("servo/pos", servohoodangle);; // Shoots at target RPM when button pressed
        } else {
            orchestrator.publish("servo/pos",  (double) 0);  // Stops shooter when button released
        }
        joinedTelemetry.addData("shooterSpeed", RPM);
        joinedTelemetry.addData("Target", orchestrator.getLatestValue("shooter/targetRPM", Double.class).orElse(0.0));
        joinedTelemetry.addData("Current", orchestrator.getLatestValue("shooter/currentRPM", Double.class).orElse(0.0));
        joinedTelemetry.addData("At RPM", orchestrator.getLatestValue("shooter/atRPM", Boolean.class).orElse(false));
        joinedTelemetry.addData("Hood Angle", servohoodangle);
        joinedTelemetry.update();
    }
}
