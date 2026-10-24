package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.autonomy.State;
import com.aaravlabs.autonomy.ftc.StateMachineOpMode;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Collections;
import java.util.List;


@Autonomous
public class CoolPedroTest extends MainPedroAuto {

    @Override
    protected Pose getStartingPose() {
        return new Pose(56, 10, Math.toRadians(90));
    }

    public void SetRoute(){
        addStep(AutoStates.depart);
        addStep(AutoStates.shoot);
        addStep(AutoStates.flower);
        addStep(AutoStates.garden);
        addStep(AutoStates.end);
    }
}
