package org.firstinspires.ftc.teamcode.Subsystems;
import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.RunPeriodically;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "Field Centric Drive")
public class FieldCentricDrive extends Node {

    private final SafeDevice<DcMotorEx> fl;
    private final SafeDevice<DcMotorEx> fr;
    private final SafeDevice<DcMotorEx> bl;
    private final SafeDevice<DcMotorEx> br;


    public FieldCentricDrive(
            Orchestrator orchestrator,
            SafeDevice<DcMotorEx> frontLeft,
            SafeDevice<DcMotorEx> frontRight,
            SafeDevice<DcMotorEx> backLeft,
            SafeDevice<DcMotorEx> backRight) {
        super(orchestrator);
        this.fl = frontLeft;
        this.fr = frontRight;
        this.bl = backLeft;
        this.br = backRight;

        // Config writes have no slot, so each one replays in order instead of coalescing.
        fl.labels("write/drive").run(m -> m.setDirection(DcMotor.Direction.REVERSE));
        bl.labels("write/drive").run(m -> m.setDirection(DcMotor.Direction.REVERSE));
        fr.labels("write/drive").run(m -> m.setDirection(DcMotor.Direction.FORWARD));
        br.labels("write/drive").run(m -> m.setDirection(DcMotor.Direction.FORWARD));

        fl.labels("write/drive").run(m -> m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        bl.labels("write/drive").run(m -> m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        fr.labels("write/drive").run(m -> m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        br.labels("write/drive").run(m -> m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

    }
    @RunPeriodically(hz = 50, hardware = true)
    public void FieldCentric() {
        double y = -orchestrator
                .getLatestValue("g1/left_stick_y", Float.class)
                .map(Float::doubleValue)
                .orElse(0.0);
        double x = orchestrator
                .getLatestValue("g1/left_stick_x", Float.class)
                .map(Float::doubleValue)
                .orElse(0.0) * 1.1;
        double r = orchestrator
                .getLatestValue("g1/right_stick_x", Float.class)
                .map(Float::doubleValue)
                .orElse(0.0) * 0.7;



        double botHeading = orchestrator.getLatestValue("robot/pose/headingRad", Double.class).orElse(0.0);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1);
        double frontLeftPower = (rotY + rotX + r) / denominator;
        double backLeftPower = (rotY - rotX + r) / denominator;
        double frontRightPower = (rotY - rotX - r) / denominator;
        double backRightPower = (rotY + rotX - r) / denominator;

        fl.raw().setPower(frontLeftPower);
        bl.raw().setPower(backLeftPower);
        fr.raw().setPower(frontRightPower);
        br.raw().setPower(backRightPower);

        fl.labels("write/drive").slot("drive/fl").run(m -> m.setPower(frontLeftPower));
        fr.labels("write/drive").slot("drive/fr").run(m -> m.setPower(frontRightPower));
        bl.labels("write/drive").slot("drive/bl").run(m -> m.setPower(backLeftPower));
        br.labels("write/drive").slot("drive/br").run(m -> m.setPower(backRightPower));

        orchestrator.publish("drive/power/fl", frontLeftPower);
        orchestrator.publish("drive/power/fr", frontRightPower);
        orchestrator.publish("drive/power/bl", backLeftPower);
        orchestrator.publish("drive/power/br", backRightPower);
    }
}