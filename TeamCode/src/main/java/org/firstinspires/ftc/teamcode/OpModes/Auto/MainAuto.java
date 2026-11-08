package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.AbstractState;
import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.Submachine;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class MainAuto extends StateMachineOpMode {

    public enum AutoStates{
        shootPreload,
        leaveScoring,
        leaveAudience,
        leavePushAlliance,
        goToGarden,
        goToGardenShooting,
        pickFromFlowerAudience,
        pickFromFlowerScoring,
        parkScoring,
        parkAudience,
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
            case leaveScoring:
                return new LeaveScoringState();
            case leaveAudience:
                return new LeaveAudienceState();
            case leavePushAlliance:
                return new LeavePushAlliance();
            case goToGarden:
                return GardenSub(false);
            case goToGardenShooting:
                return GardenSub(true);
            case pickFromFlowerScoring:
                return FlowerScoringSub();
            case pickFromFlowerAudience:
                return FlowerAudienceSub();
            case parkScoring:
                return new ParkScoringState();
            case parkAudience:
                return new ParkAudienceState();


        }
        return null;

    }

    private final class ShootPreloadState extends AbstractState {

        boolean isDone = true;

        @Override
        public void init() {
            setEndCondition(() -> isDone);
        }

        @Override
        public void loop() {

        }
    }

    private final class LeaveScoringState extends AbstractState {
        Path leavepath;

        @Override
        public void init() {
            leavepath = Paths.line(lastPath, poses.redScoringLeave).constant(lastPath);
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
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath = poses.redAudienceLeave;
        }
    }

    private final class LeaveAudienceState extends AbstractState {
        Path leavepath;

        @Override
        public void init() {
            leavepath = Paths.line(lastPath, poses.redAudienceLeave).constant(lastPath);
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
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath = poses.redAudienceLeave;
        }
    }

    private final class LeavePushAlliance extends AbstractState {
        Path leavepath;

        @Override
        public void init() {
            leavepath = Paths.line(lastPath, poses.redParkingZoneAlliancePush).constant(lastPath);
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
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath = poses.redParkingZoneAlliancePush;
        }
    }

    private Submachine FlowerScoringSub(){

        return new Submachine("Flower",
                Arrays.asList(
                        new ToScoringFlowerState(),
                        new WaitState(1),
                        new LeaveScoringFlowerState()
                )
        );
    }


    private final class ToScoringFlowerState extends AbstractState {

        Path goToFlower;
        Path leaveFlower;

        public void init() {
            if(lastPath.y() > 72) goToFlower = Paths.line(lastPath, poses.redScoringFlowerIntake).constant(Math.toRadians(90));
            else goToFlower = Paths.curve(lastPath,poses.firstFlowerControl1,poses.redScoringFlowerIntake).linear(lastPath,poses.redScoringFlowerIntake);
            setEndCondition(()-> !follower.following());
            follower.follow(goToFlower);
        }

        @Override
        public void loop() {
            follower.update();
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath =  poses.redAudienceFlowerIntake;
        }
    }

    private final class LeaveScoringFlowerState extends AbstractState {

        Path leaveFlower;
        public void init() {
            leaveFlower = Paths.line(lastPath, poses.redScoringFlowerLeave).constant(Math.toRadians(90));
            setEndCondition(()-> !follower.following());
            follower.follow(leaveFlower);
        }

        @Override
        public void loop() {
            follower.update();
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath =  poses.redAudienceFlowerLeave;
        }
    }

    private Submachine FlowerAudienceSub(){

        return new Submachine("Flower",
                Arrays.asList(
                        new ToAudienceFlowerState(),
                        new WaitState(1),
                        new LeaveAudienceFlowerState()
                )
        );
    }


    private final class ToAudienceFlowerState extends AbstractState {

        Path goToFlower;
        Path leaveFlower;

        public void init() {
            if(lastPath.y() > 72) goToFlower = Paths.line(lastPath, poses.redAudienceFlowerIntake).constant(Math.toRadians(90));
            else goToFlower = Paths.curve(lastPath,poses.firstFlowerControl1,poses.redAudienceFlowerIntake).linear(lastPath,poses.redAudienceFlowerIntake);
            setEndCondition(()-> !follower.following());
            follower.follow(goToFlower);
        }

        @Override
        public void loop() {
            follower.update();
        }

        @Override
        public void stop(){
            lastPath =  poses.redAudienceFlowerIntake;
        }
    }

    private final class LeaveAudienceFlowerState extends AbstractState {

        Path leaveFlower;
        public void init() {
            leaveFlower = Paths.line(lastPath, poses.redScoringFlowerLeave).constant(Math.toRadians(90));
            setEndCondition(()-> !follower.following());
            follower.follow(leaveFlower);
        }

        @Override
        public void loop() {
            follower.update();
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath =  poses.redAudienceFlowerLeave;
        }
    }

    private Submachine GardenSub(boolean shooting){
        double shooter;
        if(shooting)shooter = 0.5;
        else shooter = 0;
        return new Submachine("Garden",
                Arrays.asList(
                        new GotoGarden(),
                        new WaitState(shooter),
                        new CollectGarden()
                )
        );
    }

    private final class GotoGarden extends AbstractState {
        Path leavepath;

        @Override
        public void init() {
            leavepath = Paths.line(lastPath, poses.redToGarden).linear(lastPath,poses.redToGarden);
            setEndCondition(() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            follower.follow(leavepath);
        }

        @Override
        public void loop() {
            follower.update();
            telemetry.addData("isdone?", Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            telemetry.addData("Velocity", follower.tangentialVelocity());
            telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
            telemetry.addData("distance to end", follower.distanceToEndpoint());
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop() {
            lastPath = poses.redToGarden;
        }
    }

    private final class CollectGarden extends AbstractState {
        Path leavepath;

        @Override
        public void init() {
            leavepath = Paths.line(lastPath, poses.redPickupGarden).constant(lastPath);
            setEndCondition(() -> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            follower.follow(leavepath);
        }

        @Override
        public void loop() {
            follower.update();
            telemetry.addData("isdone?", Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            telemetry.addData("Velocity", follower.tangentialVelocity());
            telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
            telemetry.addData("distance to end", follower.distanceToEndpoint());
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop() {
            lastPath = poses.redPickupGarden;
        }
    }

    private final class ParkScoringState extends AbstractState {
        Path parkPath;

        @Override
        public void init() {
            parkPath = Paths.line(lastPath, poses.redScoringPark).constant(lastPath);
            setEndCondition(()-> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            follower.follow(parkPath);
        }
        @Override
        public void loop() {
            follower.update();
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath = poses.redScoringPark;
            follower.stop();
        }
    }

    private final class ParkAudienceState extends AbstractState {
        Path parkPath;

        @Override
        public void init() {
            parkPath = Paths.line(lastPath, poses.redAudiencePark).constant(lastPath);
            setEndCondition(()-> Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            follower.follow(parkPath);
        }
        @Override
        public void loop() {
            follower.update();
            Poses.finalPose = follower.pose();
        }

        @Override
        public void stop(){
            lastPath = poses.redAudiencePark;
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
