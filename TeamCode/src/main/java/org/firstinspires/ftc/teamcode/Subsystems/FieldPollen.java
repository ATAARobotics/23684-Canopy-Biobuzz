package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class FieldPollen extends Node {
    /**
     * Create a node bound to the given orchestrator. Every subclass constructor must
     * call {@code super(orchestrator)}.
     *
     * @param orchestrator the bus this node publishes to and reads from
     */

    private static double pollenDia = 2.7; // average diameter

    private static double focalLengthX = 909.770; // defult fov x

    private static double CameraForward = 9.5;
    private static double CameraLeft = 2; // check that

    private static double CameraUp = 8.5;

    SafeDevice<Limelight3A> limelight;

    Follower follower;

    volatile ArrayList<Point> pollenlist;

    JoinedTelemetry joinedTelemetry;

    Set<Point> seenpollen;

    public boolean seeAnyPollen;
    public FieldPollen(Orchestrator orchestrator, SafeDevice<Limelight3A> limelight, Follower follower, JoinedTelemetry joinedTelemetry) {
        super(orchestrator);
        this.limelight = limelight;
        this.follower = follower;

        pollenlist = new ArrayList<>();
        seenpollen = new HashSet<>();

        limelight.run((l) -> l.pipelineSwitch(1));
        limelight.run((l) -> l.start());

        this.joinedTelemetry = joinedTelemetry;

        joinedTelemetry.addLine("init");


    }

    private double angleWrap(double angle) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle))));
    }

    public double[] boundingBox(List<List<Double>> corners){
        if (corners == null){
            return  null;
        }
        double minX = Double.NEGATIVE_INFINITY, maxX = Double.POSITIVE_INFINITY;
        double minY = Double.NEGATIVE_INFINITY, maxY = Double.POSITIVE_INFINITY; // to infinity and beyond

        for (List<Double> corner : corners) {
            if (corner == null || corner.size() < 2) {
                return null;
            }
            double px = corner.get(0);
            double py = corner.get(1);

            minX = Math.min(minX,px);
            maxX = Math.max(maxX,px);

            minY = Math.min(maxY,py);
            maxY = Math.max(maxY,py);
        }
    return new double[] {maxX - minX, maxY - minY};
    }

    private Point projectPollen(LLResultTypes.DetectorResult detection, Pose robotPose) {

        double[] box = boundingBox(detection.getTargetCorners());

        double width = box[0];
        double height = box[1];

//        if (width < MIN_BOX_WIDTH_PX || height < MIN_BOX_WIDTH_PX) {
//            return null;
//        }
        double distance = pollenDia * focalLengthX / width;

        double tx = Math.toRadians(
                detection.getTargetXDegreesNoCrosshair());

        double cameraRight = distance * Math.sin(tx);
        double cameraForward = distance * Math.cos(tx);


        double groundDistance = Math.sqrt(
                Math.max(0,
                        distance * distance - 1.35 * 1.35));

        cameraForward = groundDistance * Math.cos(tx);
        cameraRight = groundDistance * Math.sin(tx);

        double robotForward = CameraUp + cameraForward;
        double robotLeft = CameraLeft - cameraRight;

        double heading = robotPose.heading();

        double fieldX = robotPose.x()
                + robotForward * Math.cos(heading)
                - robotLeft * Math.sin(heading);

        double fieldY = robotPose.y()
                + robotForward * Math.sin(heading)
                + robotLeft * Math.cos(heading);

        return new Point(fieldX, fieldY);
    }

    @RunPeriodically(hz = 50)
    public void GetAllPollen() {
        LLResult llResult;
        try {
            llResult = limelight.call((l) -> l.getLatestResult());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        if (llResult != null && llResult.isValid()) {

            List<LLResultTypes.DetectorResult> detectorResults = llResult.getDetectorResults();

            if(!detectorResults.isEmpty() && detectorResults.size() > 2) pollenlist.clear();
            if(detectorResults.isEmpty()) seeAnyPollen = false;
            else seeAnyPollen = true;

            for (LLResultTypes.DetectorResult result : detectorResults) {

                Point pose = projectPollen(result,follower.pose());
                    if(seenpollen.add(pose)){
                        pollenlist.add(pose);
                    }
                    pollenlist.removeIf((p) -> p.getX() >= 72); // anything on blue side is no no
            }
        }

        if(!pollenlist.isEmpty()){
            joinedTelemetry.addData("all pollen:", pollenlist.toString());
        }

        orchestrator.publish("limelight/pollen", pollenlist);
    }
}
