package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.SafeOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

public abstract class MainPedroTestAuto extends SafeOpMode {

    public enum AutoStates{
        init,
        shootpreload,
        leave,
        pickupFlower,
        park,
        shoot,
        end
    }

    public List <AutoStates> actions;

    AutoStates autoStates;
    int step;
    int Substep = 0;

    Poses poses;
    Pose lastPath;

    Follower follower;

    Path leave;

    Path flower;
    Path leaveflower;


    @Override
    public void onSafeInit() {
        actions = new ArrayList<>();
        autoStates = AutoStates.init;
        SetRoute();
        actions.add(AutoStates.end);
        follower = Constants.create(safeMap);
        poses = new Poses(follower);
        lastPath = getStartingPose();
        follower.setPose(getStartingPose());

        leave = Paths.line(getStartingPose(),poses.leaveStart).linear(lastPath.heading(),Math.toRadians(270));
        flower = Paths.line(poses.leaveStart,poses.pickupFlower).linear(lastPath.heading(),Math.toRadians(90));
        leaveflower = Paths.line(poses.pickupFlower,poses.leaveFlower).constant(Math.toRadians(90));
    }


    protected abstract Pose getStartingPose();


    protected void SetRoute(){}


    protected void addStep(AutoStates step) {
        actions.add(step);
    }

//    private boolean donePath(){
//        if (follower != null) {
//            return Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4;
//        }else{
//            return false;
//        }
//    }
    private void updateState(){
        telemetry.addLine("updateState called with autoStates: " + autoStates.toString());
        switch (autoStates){
            case init:
                step = 0;
                autoStates = actions.get(step);
                telemetry.addLine("init");
                break;
            case shootpreload:
                telemetry.addLine("I shot my totally real preloaded pollen");
                getNextState();
                break;
            case leave:
                follower.follow(leave);
                if(follower.atParametricEnd()){
                   getNextState();
                    lastPath = poses.leaveStart;
                }
                break;
            case pickupFlower:
                boolean done = false;
                Path flower = Paths.line(lastPath,poses.pickupFlower).linear(lastPath.heading(),Math.toRadians(90));
                Path leaveflower = Paths.line(poses.pickupFlower,poses.leaveFlower).constant(Math.toRadians(90));
                switch (Substep){
                    case 0:
                        follower.follow(flower);
                        if(follower.atParametricEnd()) Substep = 1;
                        break;
                    case 1:
                        follower.follow(leaveflower);
                        if(follower.atParametricEnd()) Substep = 2;
                       break;
                    case 2:
                        done = true;
                        lastPath = poses.leaveFlower;
                        break;
                }
                if (done) getNextState();
                break;
            case park:
                Path park = Paths.line(lastPath,poses.park).constant(lastPath.heading());
                follower.follow(park);
                if(follower.atParametricEnd()){
                    lastPath = poses.park;
                    getNextState();
                }
                break;
            case shoot:
                getNextState();
                autoStates = actions.get(step);
                telemetry.addLine("I shot my totally real pollen");
                break;

            case end:
                break;
            default:
                throw new IllegalArgumentException("Action unknown!");
        }
        telemetry.addLine("updateState exiting with autoStates: " + autoStates.toString());

    }

    private void getNextState(){
        telemetry.addLine("getNextState called with step: " + step);
        boolean checked = false;
        if(!checked){
            telemetry.addLine("CurrentState = " + actions.get(step).toString());
            step += 1;
            Substep = 0;
            autoStates = actions.get(step);
            telemetry.addLine("NextState = " + actions.get(step).toString());
            checked = true;
        }
        telemetry.addLine("getNextState exiting with step: " + step);
    }

    @Override
    public void  onSafeLoop() {
        updateState();
        telemetry.addData("Current State",autoStates.toString());
        telemetry.addData("at end?",follower.atParametricEnd());
        telemetry.addData("busy?", !follower.isBusy());
       // telemetry.addData("Done Path",donePath());
        telemetry.update();
        follower.update();
    }
}
