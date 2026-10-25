package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.AbstractState;
import com.aaravlabs.autonomy.HoldState;
import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class otherMainPedroAuto extends StateMachineOpMode {

    public enum AutoStates{
        shootPreload,
        leave,
        pickFromFlower,
        park,
        wait
    }

    List<State> autoStates;
    Follower follower;

    Pose lastPath;

    Poses poses;
    public void SetRoute(){}

    public void addStep(AutoStates autostates){
        autoStates.add(GetState(autostates));
    }

    public void addStep(int wait){
        autoStates.add(new WaitState(wait));
    }

    protected abstract Pose getStartingPose();


    private State GetState(AutoStates autoStates){

        switch (autoStates){
            case shootPreload:
                return new ShootPreloadState();
            case leave:
                return new LeaveState();
            case pickFromFlower:
                return FlowerSub();
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
            leavepath = Paths.line(lastPath, poses.leaveStart).constant(lastPath);
            setEndCondition(()-> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            follower.follow(leavepath);
        }
        @Override
        public void loop() {
            follower.update();
            telemetry.addData("isdone?",Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            telemetry.addData("Velocity",follower.tangentialVelocity());
            telemetry.addData("Predicted Velocity",Constants.foresightConfig.velocityConstraint.get());
            telemetry.addData("distance to end", follower.distanceToEndpoint());
        }

        @Override
        public void stop(){
            lastPath = poses.leaveStart;
        }
    }

    private Submachine FlowerSub(){

        return new Submachine("Flower",
                Arrays.asList(
                        new ToFlowerState(),
                        new WaitState(1),
                        new LeaveFlowerState()
                )
        );
    }


    private final class ToFlowerState extends AbstractState {

        Path goToFlower;
        Path leaveFlower;

        public void init() {
            goToFlower = Paths.line(lastPath, poses.pickupFlower).constant(Math.toRadians(90));
            setEndCondition(()-> !follower.following());
            follower.follow(goToFlower);
        }

        @Override
        public void loop() {
            follower.update();
        }

        @Override
        public void stop(){
            lastPath =  poses.pickupFlower;
        }
    }

    private final class LeaveFlowerState extends AbstractState {

        Path leaveFlower;
        public void init() {
            leaveFlower = Paths.line(lastPath, poses.leaveFlower).constant(Math.toRadians(90));
            setEndCondition(()-> !follower.following());
            follower.follow(leaveFlower);
        }

        @Override
        public void loop() {
            follower.update();
        }

        @Override
        public void stop(){
            lastPath =  poses.leaveFlower;
        }
    }

//    private final class FlowerState extends AbstractState {
//
//        int step = 0;
//        Path flowerPath;
//
//        Path pickFlowerPath;
//        Path leaveFlower;
//        boolean isDone = false;
//
//        boolean ranpath1 = false;
//        boolean ranpath2 = false;
//        @Override
//        public void init() {
//            flowerPath = Paths.line(lastPath, poses.pickupFlower).constant(Math.toRadians(90));
//            leaveFlower = Paths.line(poses.pickupFlower, poses.leaveFlower).constant(Math.toRadians(90));
//            setEndCondition(()->isDone);
//        }
//        @Override
//        public void loop() {
//            follower.update();
//            switch (step){
//                case 0:
//                    if(!ranpath1) {
//                        follower.follow(flowerPath);
//                        ranpath1 = true;
//                    }
//                    if(!follower.isBusy()) {
//                        follower.hold(poses.pickupFlower,true);
//                        step = 1;
//                    }
//                    break;
//                case 1:
//                    if(!ranpath2) {
//                        follower.follow(leaveFlower);
//                        ranpath2 = true;
//                    }
//                    if(!follower.isBusy()) {
//                        follower.hold(poses.leaveFlower,true);
//                        step = 2;
//                    }
//                case 2:
//                    isDone = true;
//            }
//        }
//
//        @Override
//        public void stop(){
//            lastPath = poses.leaveFlower;
//        }
//
//
//    }

    private final class ParkState extends AbstractState {
        Path parkPath;

        @Override
        public void init() {
            parkPath = Paths.line(lastPath, poses.park).constant(lastPath);
            setEndCondition(()-> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            follower.follow(parkPath);
        }
        @Override
        public void loop() {
            follower.update();
        }

        @Override
        public void stop(){
            lastPath = poses.park;
            follower.stop();
        }
    }


    protected List<State> buildStates() {

        follower = Constants.create(safeMap);
        autoStates = new ArrayList<>();
        SetRoute();
        follower.setPose(getStartingPose());
        lastPath = getStartingPose();
        poses = new Poses(follower);
//        orchestrator.registerNode("shooter", new Intake(orchestrator, shooter));
        follower.holdEnd.set(true);

        return autoStates;
    }
}
