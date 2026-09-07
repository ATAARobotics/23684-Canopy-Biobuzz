package org.firstinspires.ftc.teamcode;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.OnHardwareThread;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.GamepadAdaptor;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.aaravlabs.synapse.ftc.SafeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "PubsubSmokeTest", group = "Test")
public class PubsubSmokeTest extends SafeOpMode {

    @Override
    protected void onSafeInit() {
        SafeDevice<DcMotorEx> frontLeft  = safeMap.device(DcMotorEx.class, "frontLeft");
        SafeDevice<DcMotorEx> frontRight = safeMap.device(DcMotorEx.class, "frontRight");
        SafeDevice<DcMotorEx> backLeft   = safeMap.device(DcMotorEx.class, "backLeft");
        SafeDevice<DcMotorEx> backRight  = safeMap.device(DcMotorEx.class, "backRight");

        orch.registerNode("drive", new DriveNode(orch,
                frontLeft, frontRight, backLeft, backRight));

        GamepadAdaptor.attach(orch, gamepad1, "g1");

        hardware.bulkRead(50, view -> {
            VoltageSensor vs = hardwareMap.get(VoltageSensor.class, "voltage_sensor");
            if (vs != null) view.publish("battery/v", vs.getVoltage());
        });
    }

    @Override
    protected void onSafeLoop() {
        telemetry.addData("frontLeft",  orch.getLatestValue("drive/frontLeft",  Double.class).orElse(0.0));
        telemetry.addData("frontRight", orch.getLatestValue("drive/frontRight", Double.class).orElse(0.0));
        telemetry.addData("backLeft",   orch.getLatestValue("drive/backLeft",   Double.class).orElse(0.0));
        telemetry.addData("backRight",  orch.getLatestValue("drive/backRight",  Double.class).orElse(0.0));
        Double battery = orch.getLatestValue("battery/v", Double.class).orElse(null);
        if (battery != null) telemetry.addData("battery", battery);
        telemetry.update();
    }

    public static class DriveNode extends Node {
        private final SafeDevice<DcMotorEx> frontLeft;
        private final SafeDevice<DcMotorEx> frontRight;
        private final SafeDevice<DcMotorEx> backLeft;
        private final SafeDevice<DcMotorEx> backRight;

        DriveNode(Orchestrator orch,
                  SafeDevice<DcMotorEx> frontLeft,
                  SafeDevice<DcMotorEx> frontRight,
                  SafeDevice<DcMotorEx> backLeft,
                  SafeDevice<DcMotorEx> backRight) {
            super(orch);
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }

        @SubscribedTo(topic = "g1/a/rising")
        public void onAPress(Boolean v)   { orchestrator.publish("drive/frontLeft",  1.0); }
        @SubscribedTo(topic = "g1/a/falling")
        public void onARelease(Boolean v) { orchestrator.publish("drive/frontLeft",  0.0); }

        @SubscribedTo(topic = "g1/b/rising")
        public void onBPress(Boolean v)   { orchestrator.publish("drive/frontRight", 1.0); }
        @SubscribedTo(topic = "g1/b/falling")
        public void onBRelease(Boolean v) { orchestrator.publish("drive/frontRight", 0.0); }

        @SubscribedTo(topic = "g1/x/rising")
        public void onXPress(Boolean v)   { orchestrator.publish("drive/backLeft",   1.0); }
        @SubscribedTo(topic = "g1/x/falling")
        public void onXRelease(Boolean v) { orchestrator.publish("drive/backLeft",   0.0); }

        @SubscribedTo(topic = "g1/y/rising")
        public void onYPress(Boolean v)   { orchestrator.publish("drive/backRight",  1.0); }
        @SubscribedTo(topic = "g1/y/falling")
        public void onYRelease(Boolean v) { orchestrator.publish("drive/backRight",  0.0); }

        @OnHardwareThread
        @SubscribedTo(topic = "drive/frontLeft")
        public void fl(double p) { frontLeft.run(m -> m.setPower(p)); }
        @OnHardwareThread
        @SubscribedTo(topic = "drive/frontRight")
        public void fr(double p) { frontRight.run(m -> m.setPower(p)); }
        @OnHardwareThread
        @SubscribedTo(topic = "drive/backLeft")
        public void bl(double p) { backLeft.run(m -> m.setPower(p)); }
        @OnHardwareThread
        @SubscribedTo(topic = "drive/backRight")
        public void br(double p) { backRight.run(m -> m.setPower(p)); }
    }
}
