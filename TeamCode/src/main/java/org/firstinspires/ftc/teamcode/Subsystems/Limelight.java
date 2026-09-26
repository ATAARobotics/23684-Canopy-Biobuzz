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

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Limelight extends Node {

    SafeDevice<Limelight3A> limelight;

    Telemetry telemetry;



    public Limelight(Orchestrator orch, SafeDevice<Limelight3A> limelight, Telemetry telemetry) {
        super(orch);
        this.limelight = limelight;
        this.telemetry = telemetry;

        limelight.run(limelight3A -> limelight3A.start());
        limelight.run( limelight3A -> limelight3A.pipelineSwitch(1));
    }

    @RunPeriodically(hz = 50, hardware = true)
    public void Telemetry(){

        LLResult llResult = limelight.raw().getLatestResult();

       double closestPollen = Double.NEGATIVE_INFINITY;

        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = llResult.getDetectorResults();
            for (LLResultTypes.DetectorResult result : detectorResults) {

                if(result.getClassName() == "yellow_pollen") {

                    double x = result.getTargetXDegrees();
                    telemetry.addData("Pollen: ", x);
                }
            }
            LLResultTypes.DetectorResult closestResult = detectorResults.stream().max(Comparator.comparing(LLResultTypes.DetectorResult::getTargetArea)).orElse(null);

            telemetry.addData("closestPollen",closestResult.getTargetXDegrees());

        }

    }
}