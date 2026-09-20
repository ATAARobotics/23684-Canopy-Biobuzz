package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.FeedForwardController;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;


public class Shooter extends Node {

    SafeDevice<DcMotorEx> shooter;

    double P,I,D;
    double kV, kS;

    PIDFController shooterPIDF;
    FeedForwardController shooterFF;
    double RPM;
    public static double TICKS_PER_REVOLUTION = 28.0;
    public static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
    double Target = 0.0;
    public static double STOP_POWER = 0.0;
    public boolean atRPM;
    public Shooter(Orchestrator orchestrator, SafeDevice<DcMotorEx> shooter) {
        super(orchestrator);
        this.shooter = shooter;
        shooterFF = new FeedForwardController(kV,kS,0);
        shooterPIDF = new PIDFController(P,I,D);

    }

    @RunPeriodically(hz = 50, hardware = true)
    public void Update(){
        updateRPM();
        updateMotor();
        atRPM = (Target > 50) && (Math.abs(Target - RPM) < 50);

         double RPM  = orchestrator.getLatestValue("shooter/RPM",Float.class).map(Float::doubleValue).orElse(0.0);
         SetTarget(RPM);
    }

    public void SetTarget(double target){
        Target = target;
    }

    private void updateRPM() {
        double shooterVelocity = shooter.raw().getVelocity();
        RPM = shooterVelocity * RPM_CONVERSION;
    }

    private void updateMotor(){
        double Power;

        if (Math.abs(Target) < 50) {
            Power = STOP_POWER;
        } else {
            // Only calculate PID/FF when we actually want to move
            double pid = shooterPIDF.getOutput(RPM,Target);
            double ff = shooterFF.calculate(Target, 0);
            Power = ff + pid;
        }
    }


}
