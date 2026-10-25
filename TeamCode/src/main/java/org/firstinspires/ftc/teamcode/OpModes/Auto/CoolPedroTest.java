package org.firstinspires.ftc.teamcode.OpModes.Auto;


import com.aaravlabs.safepedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public abstract class CoolPedroTest extends CoolMainAuto {

    @Override
    protected Pose getStartingPose() {
        return new Pose(9.7, 131, Math.toRadians(90));
    }

    public void SetRoute(){
        addStep(AutoStates.Depart);
        addStep(AutoStates.FirstShoot);
        addStep(AutoStates.FirstGarden);
        addStep(AutoStates.SecondShoot);
        addStep(AutoStates.FirstFlower);
        addStep(AutoStates.ThirdShoot);
        addStep(AutoStates.SecondFlower);
        addStep(AutoStates.FourthShoot);
        addStep(AutoStates.End);
        // depart -> shoot -> garden -> shoot -> flower -> shoot -> flower -> shoot -> park
    }

    private void addStep(AutoStates autoStates) {
    }
}
