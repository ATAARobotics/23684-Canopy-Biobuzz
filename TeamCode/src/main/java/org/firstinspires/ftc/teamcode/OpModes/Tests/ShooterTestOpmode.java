package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class ShooterTestOpmode extends SafeOpMode {

    private SafeDevice<DcMotorEx> shooter;

    public static int RPM;

    @Override
    protected void onSafeInit() {
        shooter = safeMap.device(DcMotorEx.class, "shooter");

        orch.registerNode("shooter", new Shooter(orch,shooter));
    }

    @Override
    protected void onSafeLoop() {

        telemetry.addData("shooterSpeed",RPM);
    }
}
