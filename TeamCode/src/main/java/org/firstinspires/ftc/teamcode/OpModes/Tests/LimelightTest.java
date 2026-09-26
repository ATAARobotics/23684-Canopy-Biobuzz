package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Subsystems.Limelight;

public class LimelightTest extends SafeOpMode {

    SafeDevice<Limelight3A> limelight;

    @Override
    protected void onSafeInit() {
        limelight = safeMap.device(Limelight3A.class,"limelight");
        orch.registerNode("limelight",new Limelight(orch,limelight));
    }

    protected void onSafeLoop(){
    }
}
