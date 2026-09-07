package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.aaravlabs.synapse.ftc.SafeOpMode;

import java.util.ArrayList;
import java.util.List;

public class MainTestAuto extends SafeOpMode {

    public List <AutoStates> actions;
    
    AutoStates autoStates;
    int step;


    @Override
    public void onSafeInit() {
        actions = new ArrayList<>();
        autoStates = AutoStates.init;
        SetRoute();
        actions.add(AutoStates.end);
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
                telemetry.addData("step","init");
                telemetry.addLine();
                break;
            case Shootpreload:
                step += 1;
                autoStates = actions.get(step);
                telemetry.addData("step","shootpreload");
                telemetry.addLine();
                break;
             case PickupSpikeOne:
                 step += 1;
                 autoStates = actions.get(step);
                 telemetry.addData("step","pickup");
                 telemetry.addLine();
                 break;
             case Shoot:
                 step += 1;
                 autoStates = actions.get(step);
                 telemetry.addData("step","shoot");
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
    }
}
