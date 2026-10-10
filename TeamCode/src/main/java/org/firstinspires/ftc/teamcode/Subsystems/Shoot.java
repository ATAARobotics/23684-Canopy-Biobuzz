package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.FeedForwardController;
import org.firstinspires.ftc.teamcode.Utils.PIDController;


public class Shoot extends Node {

    private final SafeDevice<DcMotorEx> shooter;
    double P = 0.013, I = 0, D = 0;
    double kV = 0.0003, kS = 0.035;
    PIDController shooterPIDF;
    FeedForwardController shooterFF;

    public static double TICKS_PER_REVOLUTION = 28.0;
    public static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
    double Target = 0.0;
    double RPM = 0.0;
    public static double STOP_POWER = 0.0;
    public boolean atRPM;
    public static final double velocityCoefficient = 1/360.0 * 60.0 ;

    public Shoot(Orchestrator orchestrator, SafeDevice<DcMotorEx> shooter) {
        super(orchestrator);
        this.shooter = shooter;
        shooterFF = new FeedForwardController(kS, kV, 0);
        shooterPIDF = new PIDController(P, I, D);
        shooter.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
    }

    @RunPeriodically(hz = 50, hardware = true)
    public void update() {
        double targetRPM = orchestrator.getLatestValue("shooter/RPM", Float.class).map(Float::doubleValue).orElse(0.0);
        setTarget(targetRPM);
        updateRPM();
        updateMotor();
        atRPM = (Target > 80) && (Math.abs(Target - RPM) < 50);



        orchestrator.publish("shooter/currentRPM", RPM);
        orchestrator.publish("shooter/targetRPM", Target);
        orchestrator.publish("shooter/atRPM", atRPM);
    }

    private void updateRPM() {
        shooter.run(m -> {
            double velocity = m.getVelocity(AngleUnit.DEGREES); // degrees per second
            // Conversion: (deg/s) / 360 = rev/s. rev/s * 60 = RPM
            RPM = velocity * velocityCoefficient;
        });
    }

    private void updateMotor() {
        if (Target < 80) {
            shooter.run(m -> m.setPower(STOP_POWER));
            shooterPIDF.reset();
        } else {
            double pidOutput = shooterPIDF.calculate(RPM,Target);
            double ffOutput = shooterFF.calculate(Target, 0);
            shooter.run(m -> m.setPower(pidOutput + ffOutput));
        }
    }

    public void setTarget(double target) {
        this.Target = target;
    }

    @RunPeriodically(hz = 50)
    public void RunShooter() {
        double operator = -orchestrator.getLatestValue("g1/right_stick_y", Float.class).map(Float::doubleValue).orElse(0.0);
        double trigger = orchestrator.getLatestValue("g2/right_trigger", Float.class).map(Float::doubleValue).orElse(0.0);
        double power = Math.max(operator, trigger);
        if (power > 0 || Target <= 0) {
            shooter.run(m -> m.setPower(power));
        }
        orchestrator.publish("shoot/power", power);
        orchestrator.publish("shooter/power", power);
    }
}
