package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class TurretTest extends SafeOpMode {

    Follower follower;

    SafeDevice<DcMotor> encoder;

    SafeDevice<CRServo> turretServo;




    @Override
    protected void onSafeInit() {
        encoder = safeMap.device(DcMotor.class,"encoder");
        turretServo = safeMap.device(CRServo.class,"turret");
        follower = Constants.create(safeMap);
        orch.registerNode("turret",new Turret(turretServo,encoder,follower,telemetry,orchestrator));
        GamepadAdaptor.attach(orch, gamepad1, "g1");

        follower.setPose(new Pose(63,9,Math.toDegrees(90)));


    }

    @Override
    protected void onSafeLoop(){
        double y = -orchestrator.getLatestValue("g1/left_stick_y", Float.class).map(Float::doubleValue).orElse(0.0);
        double x = orchestrator.getLatestValue("g1/left_stick_x", Float.class).map(Float::doubleValue).orElse(0.0) * 1.1;
        double r = -orchestrator.getLatestValue("g1/right_stick_x", Float.class).map(Float::doubleValue).orElse(0.0);
        follower.manual(x,y,r);
        follower.update();
    }
}
