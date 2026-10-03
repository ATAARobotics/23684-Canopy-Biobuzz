package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shoot extends Node {

    private final SafeDevice<DcMotorEx> shooter;

    public Shoot(Orchestrator orchestrator, SafeDevice<DcMotorEx> shooter) {
        super(orchestrator);
        this.shooter = shooter;
    }

    @RunPeriodically(hz = 50, hardware = true)
    public void RunShooter() {
        double operator = -orchestrator.getLatestValue("g1/right_stick_y", Float.class).map(Float::doubleValue).orElse(0.0);
        double trigger = orchestrator.getLatestValue("g2/right_trigger", Float.class).map(Float::doubleValue).orElse(0.0);
        double power = Math.max(operator, trigger);
        shooter.run(m -> m.setPower(power));
        orchestrator.publish("shooter/power", power);
    }
}

