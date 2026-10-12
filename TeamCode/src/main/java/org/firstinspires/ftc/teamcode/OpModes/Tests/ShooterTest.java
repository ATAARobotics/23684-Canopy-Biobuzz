package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterTest extends Node {
    private final SafeDevice<DcMotorEx> sh;

    public ShooterTest(Orchestrator orch, SafeDevice<DcMotorEx> shooter) {
        super(orch);
        this.sh = shooter;
        sh.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
    }

    @RunPeriodically(hz = 50, hardware = true)
    public void shoot() {
        boolean isPressed = orchestrator.getLatestValue("g2/x", Boolean.class).orElse(false);

        double power = isPressed ? 1.0 : 0.0;

        sh.run(m -> m.setPower(power));

        orchestrator.publish("shooter/power", power);
    }
}
