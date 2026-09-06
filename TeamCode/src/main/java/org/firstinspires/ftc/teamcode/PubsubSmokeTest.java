package org.firstinspires.ftc.teamcode;

import com.aaravlabs.pubsub.Node;
import com.aaravlabs.pubsub.Orchestrator;
import com.aaravlabs.pubsub.annotation.OnHardwareThread;
import com.aaravlabs.pubsub.annotation.RunPeriodically;
import com.aaravlabs.pubsub.annotation.RunnableAction;
import com.aaravlabs.pubsub.annotation.SubscribedTo;
import com.aaravlabs.pubsub.ftc.GamepadAdaptor;
import com.aaravlabs.pubsub.ftc.SafeDevice;
import com.aaravlabs.pubsub.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "PubsubSmokeTest", group = "Test")
public class PubsubSmokeTest extends SafeOpMode {

    private SafeDevice<DcMotorEx> leftDrive;
    private SafeDevice<DcMotorEx> rightDrive;
    private SafeDevice<Servo> intakeServo;

    @Override
    protected void onSafeInit() {
        leftDrive = safeMap.device(DcMotorEx.class, "leftDrive");
        rightDrive = safeMap.device(DcMotorEx.class, "rightDrive");
        intakeServo = safeMap.device(Servo.class, "intakeServo");

        orch.registerNode("drive", new DriveNode(orch, leftDrive, rightDrive));
        orch.registerNode("intake", new IntakeNode(orch, intakeServo));
        orch.registerNode("safety", new SafetyNode(orch));

        GamepadAdaptor.attach(orch, gamepad1, "g1");

        hardware.bulkRead(50, view -> {
            VoltageSensor vs = hardwareMap.get(VoltageSensor.class, "voltage_sensor");
            if (vs != null) view.publish("battery/v", vs.getVoltage());
        });
    }

    @Override
    protected void onSafeLoop() {
        telemetry.addData("drive/l", orch.getLatestValue("drive/powerL", Double.class).orElse(0.0));
        telemetry.addData("drive/r", orch.getLatestValue("drive/powerR", Double.class).orElse(0.0));
        telemetry.addData("intake", orch.getLatestValue("intake/power", Double.class).orElse(0.0));
        Double battery = orch.getLatestValue("battery/v", Double.class).orElse(null);
        if (battery != null) telemetry.addData("battery", battery);
        telemetry.update();
    }

    public static class DriveNode extends Node {
        private final SafeDevice<DcMotorEx> left;
        private final SafeDevice<DcMotorEx> right;

        DriveNode(Orchestrator orch, SafeDevice<DcMotorEx> left, SafeDevice<DcMotorEx> right) {
            super(orch);
            this.left = left;
            this.right = right;
        }

        @SubscribedTo(topic = "drive/powerL")
        public void leftPower(double p) { left.run(m -> m.setPower(p)); }

        @SubscribedTo(topic = "drive/powerR")
        public void rightPower(double p) { right.run(m -> m.setPower(p)); }

        @RunPeriodically(hz = 50, hardware = true)
        public void update() {
            Double l = orchestrator.getLatestValue("g1/left_stick_y", Double.class).orElse(0.0);
            Double r = orchestrator.getLatestValue("g1/right_stick_y", Double.class).orElse(0.0);
            orchestrator.publish("drive/powerL", l);
            orchestrator.publish("drive/powerR", r);
        }
    }

    public static class IntakeNode extends Node {
        private final SafeDevice<Servo> servo;
        IntakeNode(Orchestrator orch, SafeDevice<Servo> servo) {
            super(orch);
            this.servo = servo;
        }

        @OnHardwareThread
        @SubscribedTo(topic = "intake/power")
        public void onPower(double p) { servo.run(s -> s.setPosition(p)); }

        @SubscribedTo(topic = "g1/right_bumper/rising")
        public void onBumper(Boolean v) {
            orchestrator.publish("intake/power", 1.0);
        }

        @SubscribedTo(topic = "g1/right_bumper/falling")
        public void offBumper(Boolean v) {
            orchestrator.publish("intake/power", 0.0);
        }

        @RunnableAction("shoot")
        public void shoot() {
            orchestrator.publish("intake/power", 0.5);
        }
    }

    public static class SafetyNode extends Node {
        SafetyNode(Orchestrator orch) { super(orch); }

        @RunPeriodically(hz = 10)
        public void watch() {
            Double v = orchestrator.getLatestValue("battery/v", Double.class).orElse(null);
            if (v != null && v < 11.0) {
                orchestrator.publish("drive/powerL", 0.0);
                orchestrator.publish("drive/powerR", 0.0);
            }
        }

        @SubscribedTo(topic = "g1/x")
        public void onX(Boolean v) {
            orchestrator.runAction("shoot");
        }
    }
}
