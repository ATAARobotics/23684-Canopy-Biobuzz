package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "test auto")
public class TestAuto extends MainTestAuto{

    @Override
    protected void SetRoute() {
        addStep(AutoStates.Shootpreload);
        addStep(AutoStates.PickupSpikeOne);
        addStep(AutoStates.Shoot);
    }
}
