package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.aaravlabs.safepedropathing.api.Paths.*;

import com.aaravlabs.safepedropathing.api.PoseFactory;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;

public class Poses {

    public final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startAudience = poseFactory.of(55, 9, 90);
    public final Pose point1 = poseFactory.of(55, 9.5, 90);
    public final Pose flowerIntake = poseFactory.of(12.7, 47.5, 180);
    public final Pose flowerLeave = poseFactory.of(15.7, 47.5, 180);
    public final Pose point3 = poseFactory.of(23.5, 9.5, 180);
    public final Pose pickupGarden = poseFactory.of(7.5, 9.8, 178.9258);
    public final Pose thenShoot = poseFactory.of(30, 30, 41.9168);
    public final Pose audiencePark = poseFactory.of(11.5, 92.5382, 106.4792);


    public final Pose start = poseFactory.of(60.3538, 133.5359, 270);
    public final Pose leaveStart = poseFactory.of(60.3538, 116.909, 270);
    public final Pose pickupFlower = poseFactory.of(46.2697, 127.238, 90);
    public final Pose leaveFlower = poseFactory.of(44.1718, 121.735, 90);
    public final Pose park = poseFactory.of(8.016, 100.4927, 90);


    Follower follower;

    public Poses(Follower follower){
        this.follower = follower;
    }

    public Path leaveStart() {
        return line(follower.pose(), leaveStart).tangent();
    }

    public Path pickupFlower() {
        return line(follower.pose(), pickupFlower).constant(pickupFlower);
    }

    public Path leaveFlower() {
        return line(follower.pose(), leaveFlower).constant(leaveFlower);
    }

    public Path park() {
        return line(follower.pose(), park).constant(park);
    }
}