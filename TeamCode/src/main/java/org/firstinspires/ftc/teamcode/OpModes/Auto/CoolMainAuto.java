package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.aaravlabs.safepedropathing.api.Paths.curve;

import com.aaravlabs.autonomy.AbstractState;
import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.WaitState;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.api.Paths;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.OpModes.pedropathing.Constants;

import java.util.ArrayList;
import java.util.List;

public abstract class CoolMainAuto extends StateMachineOpMode {

    public enum AutoStates {
        Depart,
        FirstShoot,
        FirstGarden,
        SecondShoot,
        FirstFlower,
        ThirdShoot,
        SecondFlower,
        FourthShoot,
        End
    }

    List<State> autoStates;
    Follower follower;

    Pose lastPath;

    Poses poses;

    public void SetRoute() {
    }


    public void addStep(MainPedroAuto.AutoStates autostates){
        autoStates.add(GetState(autostates));
    }

    protected abstract State GetState(MainPedroAuto.AutoStates autostates);


    public void addStep(int wait){
        autoStates.add(new WaitState(wait));
    }

    protected abstract Pose getStartingPose();


    private State GetState(AutoStates autoStates) {

        switch (autoStates) {
            case Depart:
                return new DepartState();
            case FirstGarden:
                return new GardenState();
            case FirstFlower:
                return new FlowerState();
            case SecondFlower:
                return new SecondFlowerState();
            case End:
                return new EndState();

        }
        return null;
    }

    private final class DepartState extends AbstractState {

        boolean isDone = true;

        @Override
        public void init() {
            setEndCondition(() -> isDone);
        }

        @Override
        public void loop() {

        }
    }

    private final class GardenState extends AbstractState {

        int step = 0;
        Path gardenCollect;
        Path secondShoot;
        boolean isDone = false;

        @Override
        public void init() {
            gardenCollect = Paths.line(lastPath, poses.gardenCollect).tangent();
            secondShoot = Paths.curve(poses.gardenCollect, poses.secondShootControl1, poses.secondShootControl2, poses.secondShoot).linear(poses.gardenCollect, poses.secondShoot);
            setEndCondition(() -> isDone);
        }

        @Override
        public void loop() {
            follower.update();
            telemetry.addData("isdone?", Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4);
            telemetry.addData("Velocity", follower.tangentialVelocity());
            telemetry.addData("Predicted Velocity", Constants.foresightConfig.velocityConstraint.get());
            telemetry.addData("distance to end", follower.distanceToEndpoint());

            follower.update();
            switch (step) {
                case 0:
                    follower.follow(gardenCollect);
                    if (Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4)
                        step = 1;
                    break;
                case 1:
                    follower.follow(secondShoot);
                    if (Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4)
                        step = 2;
                    break;
                case 2:
                    isDone = true;
            }

        }
    }

    private final class FlowerState extends AbstractState {

        int step = 0;
        Path flowerPath;
        Path thirdShoot;
        boolean isDone = false;

        @Override
        public void init() {
            flowerPath = Paths.curve(lastPath, poses.firstFlowerControl1, poses.firstFlower).constant(poses.firstFlower);
            thirdShoot = Paths.curve(poses.firstFlower, poses.thirdShootControl1, poses.thirdShoot).constant(poses.thirdShoot);
            setEndCondition(() -> isDone);
        }

        @Override
        public void loop() {
            follower.update();
            switch (step) {
                case 0:
                    follower.follow(flowerPath);
                    if (Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4)
                        step = 1;
                    break;
                case 1:
                    follower.follow(thirdShoot);
                    if (Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4)
                        step = 2;
                    break;
                case 2:
                    isDone = true;
            }
        }
    }

        private final class SecondFlowerState extends AbstractState {

            int step = 0;
            Path flowerPath;
            Path fourthShoot;
            boolean isDone = false;

            @Override
            public void init() {
                flowerPath = Paths.curve(lastPath, poses.secondFlowerControl1, poses.secondFlowerControl2, poses.secondFlower).linear(lastPath, poses.secondFlower);
                fourthShoot = Paths.curve(poses.secondFlower, poses.fourthShootControl1, poses.fourthShoot).linear(poses.secondFlower,poses.fourthShoot);
                setEndCondition(() -> isDone);
            }

            @Override
            public void loop() {
                follower.update();
                switch (step) {
                    case 0:
                        follower.follow(flowerPath);
                        if (Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4)
                            step = 1;
                        break;
                    case 1:
                        follower.follow(fourthShoot);
                        if (Math.abs(follower.tangentialVelocity()) < Constants.foresightConfig.velocityConstraint.get() && follower.distanceToEndpoint() < 4)
                            step = 2;
                        break;
                    case 2:
                        isDone = true;
                }
            }
        }
    private final class EndState extends AbstractState {
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
    protected List<State> buildStates() {

        follower = Constants.create(safeMap);
        autoStates = new ArrayList<>();
        SetRoute();
        follower.setPose(getStartingPose());
        lastPath = getStartingPose();
        poses = new Poses(follower);

        return autoStates;
    }
}

