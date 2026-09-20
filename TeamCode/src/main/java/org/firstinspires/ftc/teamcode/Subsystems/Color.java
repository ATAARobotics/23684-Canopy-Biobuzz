package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Color extends Node {
    private final SafeDevice<NormalizedColorSensor> colorSensor;

    public Color(Orchestrator orch, SafeDevice<NormalizedColorSensor> colorSensor) {
        super(orch);
        this.colorSensor = colorSensor;
    }

    @RunPeriodically(hz = 20, hardware = true)
    public void update() {
        colorSensor.run(sensor -> {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            orchestrator.publish("color/red", (double) colors.red);
            orchestrator.publish("color/green", (double) colors.green);
            orchestrator.publish("color/blue", (double) colors.blue);
            orchestrator.publish("color/alpha", (double) colors.alpha);
        });
    }
}
