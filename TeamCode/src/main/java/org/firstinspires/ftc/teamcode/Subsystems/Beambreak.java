package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Beambreak extends Node {

    // Declare the beam break sensor
    private SafeDevice <DigitalChannel> Beambreak;

    // Counter and state variables
    private int objectCount = 0;
    private boolean previousState = true; // True means unbroken (default)
    public Beambreak(Orchestrator orchestrator, SafeDevice <DigitalChannel> Beambreak) {
        // Initialize the sensor from the hardware map
        // "beam_break" must match the name configured on the Driver Station
        super(orchestrator);
        this.Beambreak = Beambreak;

        // Set the digital channel mode to INPUT to read data
        Beambreak.run(b -> b.setMode(DigitalChannel.Mode.INPUT));
    }


    @RunPeriodically(hz = 50)
    public void count() {
        // Read current sensor state:
        // true = beam unbroken, false = beam broken (object present)
        boolean currentState = Beambreak.raw().getState();

        // Detect transition: state changes from true (unbroken) to false (broken)
        if (previousState && !currentState) {
            objectCount++;
        }

        // Update the previous state for the next loop iteration
        previousState = currentState;

    }
}

