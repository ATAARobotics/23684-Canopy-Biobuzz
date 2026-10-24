package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.teamcode.Subsystems.FieldPollen;
import org.firstinspires.ftc.teamcode.Subsystems.SafePedroPathingConverter;
import org.firstinspires.ftc.teamcode.Subsystems.TheField;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Configurable
public class LimelightPedroTest extends SafeOpMode {

    Follower follower;

    TheField theField;

    SafeDevice<Limelight3A> limelight;

    JoinedTelemetry joinedTelemetry;

    boolean setpose;

    Pose currentpose;

    ArrayList<Point> pollen;

    @Override
    protected void onSafeInit() {
        limelight = safeMap.device(Limelight3A.class , "limelight");
        follower = Constants.create(safeMap);
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        orch.registerNode("turret",new FieldPollen(orch,limelight,follower,joinedTelemetry));
        GamepadAdaptor.attach(orch, gamepad1, "g1");
        theField = new TheField();
        pollen = new ArrayList<>();


        follower.setPose(new Pose(63,9,Math.toDegrees(90)));


    }

    @Override
    protected void onSafeLoop(){
        if(orch.getLatestValue("g1/a", Boolean.class).orElse(false)){

            if(setpose){
                currentpose = follower.pose();
                setpose = false;
            }

            pollen = orch.getLatestValue("limelight/pollen",ArrayList.class).orElse(new ArrayList<>());

            PathResult result = Weaver.builder()
                    .start(new Point(currentpose.x(), currentpose.y(), currentpose.heading()))
                    .addObstacle(theField.fieldBoundries)
                    .targets(pollen, point -> new Point(point.getX(),point.getY()))
                    .maxTargets(4)
                    .ordered()
                    .build();

            follower.follow(SafePedroPathingConverter.toPath(result));
        }else{
            setpose = false;
            follower.stop();
        }

        follower.update();
        joinedTelemetry.update();
    }
}