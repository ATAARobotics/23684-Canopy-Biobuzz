package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.Topic;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight extends Node {

    SafeDevice<Limelight3A> limelight;



    public Limelight(Orchestrator orch, SafeDevice<Limelight3A> limelight) {
        super(orch);
        this.limelight = limelight;

        limelight.run(limelight3A -> limelight3A.start());
        limelight.run( limelight3A -> limelight3A.pipelineSwitch(1));
    }

    @RunPeriodically(hz = 50, hardware = true)
    public void Telemetry(Telemetry telemetry){

        LLResult llResult = limelight.raw().getLatestResult();

       double closestPollen = Double.NEGATIVE_INFINITY;

        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = llResult.getDetectorResults();
            for (LLResultTypes.DetectorResult result : detectorResults) {

                if(result.getClassName() == "yellow_pollen") {

                    double x = result.getTargetXDegrees();
                    double closestPollenx = 0;

                    if(result.getTargetArea() > closestPollen){
                        double closestPolenx = result.getTargetXDegrees();
                    }


                    telemetry.addData("Pollen: ", x);
                }
            }

        }

    }
}