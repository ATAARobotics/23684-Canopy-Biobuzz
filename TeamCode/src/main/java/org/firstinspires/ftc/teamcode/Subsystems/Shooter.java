package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.FeedForwardController;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;

public class Shooter extends Node {
    private final SafeDevice<DcMotorEx> sh;

    double P = 0.013, I = 0, D = 0;
    double kV = 0.0003, kS = 0.035;

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
        this.sh = shooter;
        shooterFF = new FeedForwardController(kV, kS, 0);
        shooterPIDF = new PIDFController(P, I, D);
        sh.run(m -> m.setDirection(DcMotor.Direction.FORWARD));
    }

    @RunPeriodically(hz = 50, hardware = true)
    public void update() {
        updateRPM();
        updateMotor();
        atRPM = (Target > 50) && (Math.abs(Target - RPM) < 50);

        double targetRPM = orchestrator.getLatestValue("shooter/RPM", Float.class).map(Float::doubleValue).orElse(0.0);
        setTarget(targetRPM);

        boolean isPressed = orchestrator.getLatestValue("g2/x", Boolean.class).orElse(false);
        if (isPressed) {
            setTarget(1000.0); // Example: shoot at 1000 RPM when button pressed
        } else if (targetRPM <= 0) {
            setTarget(0.0);
        }

        orchestrator.publish("shooter/currentRPM", RPM);
        orchestrator.publish("shooter/targetRPM", Target);
        orchestrator.publish("shooter/atRPM", atRPM);
    }

    private void updateRPM() {
        sh.run(m -> {
            double velocity = m.getVelocity(AngleUnit.DEGREES); // degrees per second
            // Conversion: (deg/s) / 360 = rev/s. rev/s * 60 = RPM
            RPM = (velocity / 360.0) * 60.0;
        });
    }

    private void updateMotor() {
        if (Target <= 0) {
            sh.run(m -> m.setPower(STOP_POWER));
            shooterPIDF.reset();
        } else {
            double pidOutput = shooterPIDF.getOutput(RPM, Target);
            double ffOutput = shooterFF.calculate(Target, 0);
            sh.run(m -> m.setPower(pidOutput + ffOutput));
        }

        shooter.run( sh -> sh.setPower(Power));
    }

    public void setTarget(double target) {
        this.Target = target;
    }
}
