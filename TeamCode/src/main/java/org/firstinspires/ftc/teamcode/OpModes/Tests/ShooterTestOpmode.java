package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class ShooterTestOpmode extends SafeOpMode {

    private SafeDevice<DcMotorEx> shooter;

    public static double RPM = 1000;

    @Override
    protected void onSafeInit() {
        shooter = safeMap.device(DcMotorEx.class, "shooter");
        GamepadAdaptor.attach(orchestrator, gamepad2, "g2");

        orchestrator.registerNode("shooter", new Shooter(orchestrator, shooter));
    }

    @Override
    protected void onSafeLoop() {
        boolean isPressed = orchestrator.getLatestValue("g2/x", Boolean.class).orElse(false);
        if (isPressed) {
            orchestrator.publish("shooter/RPM",1000.0); // Example: shoot at 1000 RPM when button pressed
        } else {
            orchestrator.publish("shooter/RPM",0);
        }

        orchestrator.publish("shooter/RPM", RPM);
        telemetry.addData("shooterSpeed", RPM);
        telemetry.addData("Target", orchestrator.getLatestValue("shooter/targetRPM", Double.class).orElse(0.0));
        telemetry.addData("Current", orchestrator.getLatestValue("shooter/currentRPM", Double.class).orElse(0.0));
        telemetry.addData("At RPM", orchestrator.getLatestValue("shooter/atRPM", Boolean.class).orElse(false));

    }
}
