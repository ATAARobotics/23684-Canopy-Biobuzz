package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;

public class RedAudienceSide extends MainAuto {

    @Override
    protected Pose getStartingPose() {
        return new Pose(55, 9, Math.toRadians(90));
    }


    public void SetRoute(){
        addStep(AutoStates.leaveAudience);
        addStep(1);
        addStep(AutoStates.shootPreload);
        addStep(AutoStates.pickFromFlowerAudience);
        addStep(AutoStates.goToGardenShooting);
        addStep(AutoStates.parkAudience);

    }
}
