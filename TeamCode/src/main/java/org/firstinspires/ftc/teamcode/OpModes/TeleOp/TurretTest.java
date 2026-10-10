package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@TeleOp
public class TurretTest extends SafeOpMode {

    SafeDevice<AnalogInput> encoder;
    @Override
    protected void onSafeInit() {
        encoder = safeMap.device(AnalogInput.class, "turret");
//        orchestrator.registerNode("turret", new Turret(encoder,telemetry,orchestrator));
    }
}
