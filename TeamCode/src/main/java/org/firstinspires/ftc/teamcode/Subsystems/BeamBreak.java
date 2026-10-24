package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class BeamBreak extends Node {

    // Declare the beam break sensor
    private SafeDevice <DigitalChannel> BeamBreak;

    // Counter and state variables
    private int objectCount = 0;
    private boolean previousState = true; // True means unbroken (default)
    public BeamBreak(Orchestrator orchestrator, SafeDevice <DigitalChannel> beamBreak) {
        // Initialize the sensor from the hardware map
        // "beam_break" must match the name configured on the Driver Station
        super(orchestrator);
        this.BeamBreak = BeamBreak;

        // Set the digital channel mode to INPUT to read data
        beamBreak.run(b -> b.setMode(DigitalChannel.Mode.INPUT));
    }


      @RunPeriodically(hz = 50)
       public void count() {
          // Read current sensor state:
          // true = beam unbroken, false = beam broken (object present)
          boolean currentState = BeamBreak.raw().getState();

          // Detect transition: state changes from true (unbroken) to false (broken)
          if (previousState && !currentState) {
              objectCount++;
          }

          // Update the previous state for the next loop iteration
          previousState = currentState;

      }
    }

