package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Tester", group = "Tests")
public class ShooterTester extends SafeOpMode {

    public static double shooterPower = 0.5;
    private SafeDevice<DcMotorEx> shooter;

    @Override
    protected void onSafeInit() {
        shooter = safeMap.device(DcMotorEx.class, "shooter");

        GamepadAdaptor.attach(orchestrator, gamepad1, "g1");
    }

    @Override
    protected void onSafeLoop() {
        orchestrator.subscribe("g1/left_bumper", Boolean.class, v -> {
            if (v) {
                shooter.run(m -> m.setPower(shooterPower));
            } else {
                shooter.run(m -> m.setPower(0.0));
            }
        });

        telemetry.addData("shooter", shooterPower);
        telemetry.update();
    }
}
