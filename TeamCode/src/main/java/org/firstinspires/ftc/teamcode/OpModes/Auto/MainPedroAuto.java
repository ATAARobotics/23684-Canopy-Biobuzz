package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.AbstractState;
import com.aaravlabs.autonomy.HoldState;
import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class MainPedroAuto extends StateMachineOpMode {

    public enum AutoStates{
        shootPreload,
        leave,
        pickFromFlower,
        park
    }

    List<State> autoStates;
    Follower follower;

    Pose lastPath;
    public void SetRoute(){}

    public void addStep(AutoStates autostates){
        autoStates.add(GetState(autostates));
    }

    protected abstract Pose getStartingPose();


    @Override
    protected void onSafeInit() {
        follower = Constants.create(safeMap);
        autoStates = new ArrayList<>();
        follower.setPose(getStartingPose());
        lastPath = getStartingPose();

    }

    private State GetState(AutoStates autoStates){

        switch (autoStates){
            case shootPreload:
                return new ShootPreloadState();
            case leave:
                return new LeaveState();
            case pickFromFlower:
                return new FlowerState();
            case park:
                return new ParkState();
        }
        return null;

    }

    private final class ShootPreloadState extends AbstractState{

        boolean isDone = true;

        @Override
        public void init() {
            setEndCondition(() -> isDone);
        }

        @Override
        public void loop() {

        }
    }

    private final class LeaveState extends AbstractState {
        Path leavepath;

        @Override
        public void init() {
            leavepath = Paths.line(lastPath,)
        }
        @Override
        public void loop() {

        }
    }

    private final class FlowerState extends AbstractState {
        @Override
        public void loop() {

        }
    }

    private final class ParkState extends AbstractState {
        @Override
        public void loop() {

        }
    }


    protected List<State> buildStates() {
        return autoStates;
    }
}
