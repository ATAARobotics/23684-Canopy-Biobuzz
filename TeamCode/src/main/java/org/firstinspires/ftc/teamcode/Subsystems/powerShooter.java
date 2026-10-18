package org.firstinspires.ftc.teamcode.Subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


public class powerShooter {
    private static final double MAX_SHOOTER_POWER = 1.0;
    private static double currentShooterPower = 0.5;

    // Track previous button states for single-press detection
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevA = false;

    DcMotorEx shooterMotor;

    public powerShooter() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
    }
    public void update(Gamepad gamepad1) {
        // Right Bumper: Increase power (on single press)
        if (gamepad1.right_bumper && !prevRightBumper) {
            currentShooterPower = Math.min(MAX_SHOOTER_POWER, currentShooterPower + 0.1);
        }

        // Left Bumper: Decrease power (on single press)
        if (gamepad1.left_bumper && !prevLeftBumper) {
            currentShooterPower = Math.max(0.0, currentShooterPower - 0.1);
        }

        // A Button: Fire (on single press)
        if (gamepad1.a && !prevA) {
            shooterMotor.setPower((float) currentShooterPower);
        }

        // Update previous button states for next loop iteration
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
        prevA = gamepad1.a;
    }

    @SubscribedTo(topic = "g1/right_bumper")
    public void shooterMedium() {
        currentShooterPower = 0.5;
    }
    @SubscribedTo(topic = "g1/left_bumper")
    public void shooterLow() {
        currentShooterPower = 0.2;
    }
    @SubscribedTo(topic = "g1/b")
    public void shooterHigh() {
        currentShooterPower = 0.8;
    }
    @SubscribedTo(topic = "g1/a")
    public void Shooter(float power) {
        shooterMotor.setPower(power);
    }
    @SubscribedTo(topic = "g1/x")
    public void startProjectile() {
        // Increase power by 0.1, capped at MAX_SHOOTER_POWER (1.0)
        currentShooterPower = Math.min(MAX_SHOOTER_POWER, currentShooterPower + 0.1);

        shooterMotor.setPower(currentShooterPower);
    }

    @SubscribedTo(topic = "g1/y")
    public void stopProjectile() {
        shooterMotor.setPower(0);
    }

    }


