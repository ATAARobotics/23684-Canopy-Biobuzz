package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.AbstractState;
import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.OpModes.pedropathing.Constants;

import java.util.ArrayList;
import java.util.List;

public abstract class MainPedroAuto extends StateMachineOpMode {

    public enum AutoStates{
        shootPreload,
        leave,
        pickFromFlower,
        park,
        wait,
        pickupGarden
    }

    List<State> autoStates;
    Follower follower;

    Pose lastPath;

    Poses poses;
    SafeDevice<DcMotorEx> intakeMotor;
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
                return new FlowerState();
            case park:
                return new ParkState();
            case pickupGarden:
                return new GardenState();
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
            leavepath = Paths.line(lastPath, poses.point1).constant(poses.point1);
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
            lastPath = poses.point1;
        }
    }

    private final class FlowerState extends AbstractState {

        int step = 0;
        Path flowerPath;
        Path leaveFlower;
        boolean isDone = false;

        @Override
        public void init() {
            flowerPath = Paths.line(lastPath, poses.flowerIntake).linear(lastPath, poses.flowerIntake);
            leaveFlower = Paths.line(poses.flowerIntake, poses.flowerLeave).constant(poses.flowerIntake);
            setEndCondition(()->isDone);
        }
        @Override
        public void loop() {
            follower.update();
            telemetry.addData("step",step);
            switch (step){
                case 0:
                    follower.follow(flowerPath);
                    if(Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4) step = 1;
                    break;
                case 1:
                    follower.follow(leaveFlower);
                    intakeMotor.run((m) -> m.setPower(1));
                    if(Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4) step = 2;
                    break;
                case 2:
                    intakeMotor.run((m) -> m.setPower(0));
                    isDone = true;
                    break;
            }
        }

        @Override
        public void stop(){
            lastPath = poses.flowerLeave;
        }


    }

    private final class ParkState extends AbstractState {
        Path parkPath;
        Path halfPark;
        int step = 0;
        boolean isDone = false;
        @Override
        public void init() {
            halfPark = Paths.line(lastPath, poses.thenShoot).constant(lastPath);
            parkPath = Paths.line(poses.thenShoot, poses.audiencePark).constant(lastPath);
            setEndCondition(()-> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            follower.follow(parkPath);
        }

        @Override
        public void loop() {
            follower.update();
            switch (step){
                case 0:
                    follower.follow(halfPark);
                    if(Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4) step = 1;
                    break;
                case 1:
                    follower.follow(parkPath);
                    if(Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4) step = 2;
                    break;
                case 2:
                    isDone = true;
                    }
        }

        @Override
        public void stop(){
            lastPath = poses.park;
        }
    }
    private final class GardenState extends AbstractState {

        int step = 0;

        Path gardenPath;
        Path leaveGarden;
        boolean isDone = false;


        public void init() {
            gardenPath = Paths.line(lastPath, poses.point3).linear(lastPath, poses.point3);
            leaveGarden = Paths.line(poses.point3, poses.pickupGarden).constant(poses.pickupGarden);
            setEndCondition(()->isDone);
        }
        @Override
        public void loop() {
            follower.update();
            telemetry.addData("isdone?",Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            telemetry.addData("Velocity",follower.tangentialVelocity());
            telemetry.addData("Predicted Velocity",Constants.foresightConfig.velocityConstraint.get());
            telemetry.addData("distance to end", follower.distanceToEndpoint());

            follower.update();
            switch (step){
                case 0:

                    follower.follow(gardenPath);
                    if(Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4) step = 1;
                    break;
                case 1:
                    intakeMotor.run((m) -> m.setPower(1));
                    follower.follow(leaveGarden);
                    if(Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4) step = 2;
                    break;
                case 2:
                    intakeMotor.run((m) -> m.setPower(0));
                    isDone = true;
        }

        }


        @Override
        public void stop(){
            lastPath = poses.leaveStart;
        }
    }

    protected List<State> buildStates() {

        follower = Constants.create(safeMap);
        autoStates = new ArrayList<>();
        SetRoute();
        follower.setPose(getStartingPose());
        lastPath = getStartingPose();
        poses = new Poses(follower);
        intakeMotor = safeMap.device(DcMotorEx.class, "intake");

        return autoStates;
    }
}
