package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;

public class RedSoloAuto extends MainAuto{
    @Override
    protected Pose getStartingPose() {
        return new Pose(8,49);
    }

    @Override
    protected boolean onRed() {
        return false;
    }


    @Override
    public void SetRoute() {
        addStep(AutoStates.leavePushAlliance);
        addStep(AutoStates.goToGardenShooting);
        addStep(AutoStates.pickFromFlowerScoring);
        addStep(AutoStates.parkScoring);
    }
}
