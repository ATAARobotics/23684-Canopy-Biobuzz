package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.synapse.ftc.SafeOpMode;

import java.util.ArrayList;
import java.util.List;

public class MainPedroTestAuto extends SafeOpMode {

    public enum AutoStates{
        init,
        shootpreload,
        pickupFlower,
        park,
        shoot,
        end
    }

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
            case shootpreload:
                step += 1;
                autoStates = actions.get(step);
                telemetry.addData("step","shootpreload");
                telemetry.addLine();
                break;
            case pickupFlower:
                step += 1;
                autoStates = actions.get(step);
                telemetry.addData("step","pickup");
                telemetry.addLine();
                break;
            case shoot:
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
