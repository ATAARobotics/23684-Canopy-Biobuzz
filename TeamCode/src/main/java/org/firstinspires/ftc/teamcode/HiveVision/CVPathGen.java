package org.firstinspires.ftc.teamcode.HiveVision;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Matrix;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeHardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.SafePedroPathingConverter;
import org.firstinspires.ftc.teamcode.Subsystems.TheField;

import java.util.ArrayList;
import java.util.List;

public class CVPathGen {
    private SafeDevice<Limelight3A> ll;
    private Follower follower;
    private boolean isRed;

    CVCostAlgo algo = new CVCostAlgo();

    Telemetry telemetry;

    public ArrayList<Pose> simBall = new ArrayList<>();
    public boolean simulation = false;
    public Pose simBotPose = new Pose(0,0,0);

    TheField theField;

    ArrayList<Point> ballPoses;
    private static Matrix cameraAdjustments(Pose robotPose) {
        double heading = robotPose.heading();

        Matrix m = Matrix.rotationTransform(heading - Math.PI / 2).times(CVConstants.createRotationX(-CVConstants.CAMERA_PITCH));
        m = m.times(CVConstants.xyzConverter);
        return m;
    }

    public CVPathGen(SafeHardwareMap safeMap, Follower follower, boolean isRed, Telemetry telemetry){
        this.follower = follower;
        this.isRed = isRed;
        this.telemetry = telemetry;

        theField = new TheField();

        ballPoses = new ArrayList<>();

        ll = safeMap.device(Limelight3A.class, "limelight"); //change to your name
        ll.run((l) -> l.start());
        ll.run((l) -> l.pipelineSwitch(1)); // change to your pipeline
    }
    public CVPathGen(boolean isRed){
        this.isRed = isRed;
    }
    public ArrayList<Pose> getBalls(boolean sim){
        if(sim) return simBall;
        ArrayList<double[]> relativeBalls = new ArrayList<>();
        ArrayList<Pose> absoluteBalls = new ArrayList<>();
        LLResult result;
        try {
            result = ll.call((l) -> l.getLatestResult());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        if (result != null && result.isValid()) {
            Pose po = follower.pose();
            for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
                if (!"yellow_pollen".equalsIgnoreCase(dr.getClassName())) {
                    continue;
                }
                relativeBalls.add(new double[]{Math.toRadians(dr.getTargetXDegrees()), Math.toRadians(dr.getTargetYDegrees())});
            }
            for(double[] ball : relativeBalls){
                absoluteBalls.add(ballConverter(ball, po));
            }
        }
        drawAll(absoluteBalls);
        return absoluteBalls;
    }
    public Pose ballConverter(double[] ball, Pose curpose){
        Matrix cameraRay = new Matrix(
                new double[][] {
                        {Math.tan(ball[0])},
                        {Math.tan(ball[1])},
                        {1.0}
                }
        );
        Matrix absoluteRay = cameraAdjustments(curpose).times(cameraRay);

        double dist = -CVConstants.CAMERA_HEIGHT / absoluteRay.get(2, 0);

        Matrix cam = new Matrix(
                new double[][] {
                        {curpose.x() + CVConstants.FORWARD_OFFSET * Math.cos(curpose.heading())},
                        {curpose.y() + CVConstants.FORWARD_OFFSET * Math.sin(curpose.heading())},
                        {CVConstants.CAMERA_HEIGHT}
                }
        );
        double x = cam.get(0,0) + absoluteRay.get(0,0) * dist;
        double y = cam.get(1,0) + absoluteRay.get(1,0) * dist;
        return new Pose(x, y, 0);
    }

    private ArrayList<Point> sortBalls() {
        ArrayList<Pose> balls = getBalls(simulation);
        Pose po = simulation ? simBotPose : follower.pose();
        balls.removeIf(ball -> isRed ? ball.x() > 70 : ball.x() < 70);
        ArrayList<Point> best = new ArrayList<>();
        best.addAll(algo.costFunc(balls, po));
        return algo.combiner(best, po);
    }


    public Path pathGen() {
        ballPoses.addAll(sortBalls());

        Weaver.setConfig(new PathConfig()
                .intakeWidth(17.5 )
                .robotWidth(17.5)
                .robotHeight(18));

        PathResult result = Weaver.builder()
                .start(new Point(follower.pose().x(), follower.pose().y(), follower.pose().heading()))
                .addObstacle(theField.fieldBoundries)
                .targets(ballPoses, ballPoses -> new Point(ballPoses.getX(),ballPoses.getY()))
                .maxTargets(4)
                .ordered()
                .build();

        Path p = SafePedroPathingConverter.toPath(result);
//        ballPoses.add(follower.pose());
//        ballPoses.addAll(sortBalls());
//        if (ballPoses.size() < 2) {
//            return null;
//        }
//        Pose[] balls = ballPoses.toArray(new Pose[0]);
//        drawChosen(ballPoses);
//        Path p = Paths.through(balls).tangent();
        return p;
    }
//    public ArrayList<Pose> pathGenSim() {
//        ArrayList<Point> ballPoses = new ArrayList<>();
//        ballPoses.add(simBotPose);
//        ballPoses.addAll(sortBalls());
//        if (ballPoses.size()<2) {
//            return null;
//        }
//        Pose[] balls = ballPoses.toArray(new Pose[0]);
//        Path p = Paths.through(balls).tangent();
//        return ballPoses;
//    }

    public void drawChosen(ArrayList<Pose> balls){
        for (int i = 0; i < balls.size(); i++) {
            Pose ball = balls.get(i);
            if (i == 0) {
            } else {
            }

        }
    }
    public void drawAll(ArrayList<Pose> balls){
        for (int i = 0; i < balls.size(); i++) {
            Pose ball = balls.get(i);
        }
    }
}