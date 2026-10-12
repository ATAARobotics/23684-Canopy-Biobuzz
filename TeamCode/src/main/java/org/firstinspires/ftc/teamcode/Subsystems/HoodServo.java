package org.firstinspires.ftc.teamcode.Subsystems;

import com.aaravlabs.synapse.Node;
import com.aaravlabs.synapse.Orchestrator;
import com.aaravlabs.synapse.annotation.SubscribedTo;
import com.aaravlabs.synapse.ftc.SafeDevice;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServo extends Node {
    private final SafeDevice<Servo> hoodServo;

    public HoodServo(Orchestrator orchestrator, SafeDevice<Servo> hoodServo) {
        super(orchestrator);
        this.hoodServo = hoodServo;
        hoodServo.run(m -> m.setDirection(Servo.Direction.FORWARD));
    }

//    @SubscribedTo(topic = "g2/a/rising")
//    public void increaseAngle() throws Exception {
//        double position = hoodServo.call(Servo::getPosition) + 0.05;
//        hoodServo.run(m -> m.setPosition(position));
//        orchestrator.publish("hoodServo/position", position);
//    }
//    @SubscribedTo(topic = "g2/b/rising")
//    public void decreaseAngle() throws Exception{
//        double position = hoodServo.call(Servo::getPosition) - 0.05;
//        hoodServo.run(m -> m.setPosition(position));
//        orchestrator.publish("hoodServo/position", position);
//    }

}