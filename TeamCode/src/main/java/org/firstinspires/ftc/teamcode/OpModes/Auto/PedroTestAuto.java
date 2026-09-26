package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.ftc.SafeOpMode;

public class PedroTestAuto extends MainPedroTestAuto {

    @Override
    protected void SetRoute() {
        follower.setPose(new Pose(60.3538, 133.5359, 90));

        addStep(AutoStates.shootpreload);
        addStep(AutoStates.leave);
        addStep(AutoStates.pickupFlower);
        addStep(AutoStates.shoot);
        addStep(AutoStates.park);
    }
}
