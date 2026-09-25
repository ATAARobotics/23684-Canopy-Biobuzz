package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp
public class ShooterTestOpmode extends SafeOpMode {

    private SafeDevice<DcMotorEx> shooter;

    public static double RPM = 1000;

    @Override
    protected void onSafeInit() {
        shooter = safeMap.device(DcMotorEx.class, "shooterMotor");
        orch.registerNode("Shooter", new Shooter(orch,shooter));
    }

    @Override
    protected void onSafeLoop() {
        orch.publish("shooter/RPM", RPM);
        telemetry.addData("shooterSpeed",RPM);
    }
}
