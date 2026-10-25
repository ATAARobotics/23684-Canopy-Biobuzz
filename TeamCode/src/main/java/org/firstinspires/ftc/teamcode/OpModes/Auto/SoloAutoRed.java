package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;

public class SoloAutoRed extends otherMainPedroAuto{
    @Override
    protected Pose getStartingPose() {
        return new Pose(8,49);
    }

    @Override
    public void SetRoute() {
        addStep(AutoStates.leavePushAlliance);
        addStep(AutoStates.goToGarden);
        addStep(AutoStates.pickFromFlowerScoring);
        addStep(AutoStates.park);
    }
}
