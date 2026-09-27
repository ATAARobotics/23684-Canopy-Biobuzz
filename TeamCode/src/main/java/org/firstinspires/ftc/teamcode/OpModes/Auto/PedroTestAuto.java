package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.ftc.SafeOpMode;

public class PedroTestAuto extends MainPedroTestAuto {


    @Override
    protected Pose getStartingPose() {
        return new Pose(60.3538, 133.5359, 90);
    }

    @Override
    protected void SetRoute() {
        addStep(AutoStates.shootpreload);
        addStep(AutoStates.leave);
        addStep(AutoStates.pickupFlower);
        addStep(AutoStates.shoot);
        addStep(AutoStates.park);
    }
}
