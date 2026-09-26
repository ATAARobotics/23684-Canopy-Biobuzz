package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.ftc.SafeOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

public class MainPedroTestAuto extends SafeOpMode {

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

    Poses poses;

    Follower follower;


    @Override
    public void onSafeInit() {
        actions = new ArrayList<>();
        autoStates = AutoStates.init;
        SetRoute();
        actions.add(AutoStates.end);
        follower = Constants.create(safeMap);
        poses = new Poses(follower);
    }


    protected void SetRoute(){}


    protected void addStep(AutoStates step) {
        actions.add(step);
    }
    private void updateState(){
        switch (autoStates){
            case init:
                step = 0;
                autoStates = actions.get(step);
                telemetry.addLine("init");
                break;
            case shootpreload:
                step += 1;
                autoStates = actions.get(step);
                //telemetry.addData("step","shootpreload");
                telemetry.addLine("I shot my totally real pollen");
                telemetry.addLine();
                break;
            case leave:
                step += 1;
                follower.follow(poses.leaveStart());
                if(follower.atParametricEnd()){
                    autoStates = actions.get(step);
                    break;
                }
            case pickupFlower:
                step += 1;
                int state = 1;
                boolean done = false;
                switch (state){
                    case 1:
                        follower.follow(poses.pickupFlower());
                        if(follower.atParametricEnd()) state =2;
                        break;
                    case 2:
                        follower.follow(poses.leaveFlower());
                        if(follower.atParametricEnd()) state =3;
                    case 3:
                        done = true;
                        break;
                }
               if (done){
                   autoStates = actions.get(step);
                   break;
               }
            case park:
                step += 1;
                follower.follow(poses.park());
                if(follower.atParametricEnd()){
                    autoStates = actions.get(step);
                    break;
                }
            case shoot:
                step += 1;
                autoStates = actions.get(step);
                telemetry.addLine("I shot my totally real pollen");
                break;

            case end:
                break;
            default:
                throw new IllegalArgumentException("No actions in list!");
        }

    }

    @Override
    public void  onSafeLoop() {
        updateState();
        telemetry.addLine(actions.toString());
        follower.update();
    }
}
