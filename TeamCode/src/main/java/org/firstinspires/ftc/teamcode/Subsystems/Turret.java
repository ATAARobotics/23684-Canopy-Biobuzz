package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;

public class Turret extends Node {
    /**
     * Create a node bound to the given orchestrator. Every subclass constructor must
     * call {@code super(orchestrator)}.
     *
     * @param orch the bus this node publishes to and reads from
     */

    Follower follower;

    SafeDevice<DcMotor> encoder;

    SafeDevice<CRServo> turretServo;

    Telemetry telemetry;

    ElapsedTime elapsedTime;

    double MaxVoltage = 3.3;

    PIDFController turretPIDF;

    public static double p,i,d,f;

    double TicksToDegree = 0;

    double currentangle = 0;
    double targetangle = 0;

    public Turret(SafeDevice<CRServo> turretServo, SafeDevice<DcMotor> encoder, Follower follower, Telemetry telemetry, Orchestrator orch) {
        super(orch);
        this.encoder = encoder;
        this.telemetry = telemetry;
        this.turretServo = turretServo;
        this.follower = follower;

        turretPIDF = new PIDFController(p,i,d,f);
    }

    private double ToLocalAngle(double globalAngle){
        double heading = follower.pose().heading();

        double localangle = globalAngle - heading;
        return localangle;
    }

    public double PinpointAngle(Pose robotPose, Pose target){
        double deltaX = target.x() - robotPose.x();
        double deltaY = target.y() - robotPose.y();

        double Tanang = Math.atan2(deltaY,deltaX);
        double finalang = Tanang + Math.PI;
        return finalang;
    }

    @RunPeriodically(hz = 50, hardware = true)
    public void Update() {
        currentangle = encoder.raw().getCurrentPosition() * TicksToDegree;

        if(follower.pose().y() < 72) targetangle = ToLocalAngle(PinpointAngle(follower.pose(),new Pose(57,51)));
        else targetangle = ToLocalAngle(PinpointAngle(follower.pose(),new Pose(57,91)));

        double power = turretPIDF.getOutput(currentangle,targetangle);

        turretServo.run((s)-> s.setPower(power));

    }

    @RunPeriodically(hz = 50, hardware = true)
    public void Telemetry() {

        telemetry.addData("current Angle", currentangle);

    }
}
