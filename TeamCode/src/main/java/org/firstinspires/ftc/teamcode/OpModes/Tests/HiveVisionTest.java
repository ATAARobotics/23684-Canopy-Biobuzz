package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HiveVision.CVPathGen;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;


@TeleOp
public class HiveVisionTest extends SafeOpMode {
    private CVPathGen pather;
    private Follower follower;
    public static boolean isRed = true;
    ArrayList<Pose> allBalls = new ArrayList<>();

    boolean pathGen = false;
    @Override
    protected void onSafeInit() {
        follower = Constants.create(safeMap);
        follower.setPose(new Pose(63,9,Math.toRadians(90)));
        follower.update();
        pather = new CVPathGen(safeMap,follower, isRed, telemetry);
        GamepadAdaptor.attach(orch, gamepad1, "g1");
    }

    @Override
    protected void onSafeLoop() {
        follower.update();
        allBalls = pather.getBalls(false);
        if(orch.getLatestValue("g1/a", Boolean.class).orElse(false)){
            Path path = null;
            if (!pathGen) {
                path = pather.pathGen();
                pathGen = true;
            }
            if(path != null){
                follower.follow(path); // will not like being looped in the future for auto
            }
        }else{
            follower.stop();
            pathGen = false;
        }
        if(follower.isBusy() && follower.atParametricEnd()){
            follower.stop();
            pathGen = false;
        }
        for (Pose ball: allBalls) {
            telemetry.addLine("(" + ball.x() + ", "+ ball.y() + ")");
        }
        telemetry.addData("Robot X", follower.pose().x());
        telemetry.addData("Robot Y", follower.pose().y());
        telemetry.addData("Remaining", follower.remainingDistance());
        telemetry.addData("Dist to endpoint", follower.distanceToEndpoint());
        telemetry.addData("Completion", follower.completion());
        telemetry.update();
    }
}