package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Collections;
import java.util.List;


@Autonomous
public class PedroPathingTest extends MainPedroAuto {

    @Override
    protected Pose getStartingPose() {
        return new Pose(60.3538, 133.5359, Math.toRadians(270));
    }

    public void SetRoute(){
        addStep(AutoStates.leave);
        addStep(AutoStates.shootPreload);
        addStep(AutoStates.pickFromFlower);
        addStep(AutoStates.pickupGarden);
        addStep(AutoStates.park);
    }
}
