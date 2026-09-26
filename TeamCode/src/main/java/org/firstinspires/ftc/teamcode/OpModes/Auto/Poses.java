package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.aaravlabs.safepedropathing.api.Paths.*;

import com.aaravlabs.safepedropathing.api.PoseFactory;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;

public class Poses {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.3538, 133.5359, 90);
    private final Pose leaveStart = poseFactory.of(60.3538, 116.909, -90);
    private final Pose pickupFlower = poseFactory.of(46.2697, 127.238, 90);
    private final Pose leaveFlower = poseFactory.of(44.1718, 121.735, 90);
    private final Pose park = poseFactory.of(8.016, 100.4927, -90);

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