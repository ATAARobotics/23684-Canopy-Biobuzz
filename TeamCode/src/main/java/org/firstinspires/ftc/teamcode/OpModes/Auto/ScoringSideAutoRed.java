package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class ScoringSideAutoRed extends  otherMainPedroAuto{
    @Override
    protected Pose getStartingPose() {
        return new Pose(63,133,Math.toRadians(90));
    }

    @Override
    public void SetRoute() {
        addStep(AutoStates.leave);
        addStep(AutoStates.shootPreload);
        addStep(1);
        addStep(AutoStates.pickFromFlower);
        addStep(AutoStates.park);
    }
}
