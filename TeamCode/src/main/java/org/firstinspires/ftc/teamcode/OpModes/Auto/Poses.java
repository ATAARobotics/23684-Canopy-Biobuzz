package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.aaravlabs.safepedropathing.api.Paths.*;

import com.aaravlabs.safepedropathing.api.PoseFactory;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import com.aaravlabs.safepedropathing.paths.Path;

public class Poses {


    //TODO: Glean out repeating poses. we only need one per location.

    public final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose startAudience = poseFactory.of(55, 9, 90);
    public final Pose point1 = poseFactory.of(55, 18, 90);
    public final Pose flowerIntake = poseFactory.of(12.7, 47.5, 180);
    public final Pose flowerLeave = poseFactory.of(29.5, 30, 180);
    public final Pose point3 = poseFactory.of(23.5, 9.5, 180);
    public final Pose pickupGarden = poseFactory.of(7.5, 9.8, 180);
    public final Pose thenShoot = poseFactory.of(30, 30, 41.9168);
    public final Pose audiencePark = poseFactory.of(11.5, 92.5382, 106.4792);


    public final Pose start = poseFactory.of(60.3538, 133.5359, 270);
    public final Pose leaveStart = poseFactory.of(60.3538, 116.909, 270);
    public final Pose pickupFlower = poseFactory.of(46.2697, 127.238, 90);
    public final Pose leaveFlower = poseFactory.of(51.357346875, 106.5348046875, 90);
    public final Pose park = poseFactory.of(8.016, 100.5, 90);

    public final Pose startPoint = poseFactory.of(56, 10, 90);
    public final Pose firstShoot = poseFactory.of(56, 10, 180);
    public final Pose gardenCollect = poseFactory.of(10, 10, 180);
    public final Pose secondShootStart = poseFactory.of(10, 10, 180);
    public final Pose secondShoot = poseFactory.of(53, 124, 90);
    public final Pose firstFlower = poseFactory.of(47, 130, 90);
    public final Pose thirdShoot = poseFactory.of(53, 124, 90);
    public final Pose secondFlowerStart = poseFactory.of(53, 124, 90);
    public final Pose secondFlower = poseFactory.of(12, 47, 180);
    public final Pose fourthShootStart = poseFactory.of(12, 47, 180);
    public final Pose fourthShoot = poseFactory.of(56, 10, 90);
    public final Pose parkAutoStart = poseFactory.of(56, 10, 90);
    public final Pose parkAuto = poseFactory.of(11, 99, 270);
    // depart -> shoot -> garden -> shoot -> flower -> shoot -> flower -> shoot -> park

    public final Pose secondShootControl1 = poseFactory.of(24, 54, 0);
    public final Pose secondShootControl2 = poseFactory.of(20, 122, 0);
    public final Pose firstFlowerControl1 = poseFactory.of(46, 119, 0);
    public final Pose thirdShootControl1 = poseFactory.of(46, 119, 0);
    public final Pose secondFlowerControl1 = poseFactory.of(16, 97, 0);
    public final Pose secondFlowerControl2 = poseFactory.of(40, 47, 0);
    public final Pose fourthShootControl1 = poseFactory.of(58, 36, 0);
    public final Pose parkAutoControl1 = poseFactory.of(32, 40, 0);


    public final Pose leavepushA = poseFactory.of(7.8945, 59.4873, 90);
    public final Pose finishPush = poseFactory.of(7.8945, 82.0361, 270);
    public final Pose goToGardenStart = poseFactory.of(7.8945, 82.0361, 270);
    public final Pose goToGarden = poseFactory.of(45.3486, 9.1201, 180);
    public final Pose pickUpGarden = poseFactory.of(11.3555, 9.1201, 180);
    public final Pose toFlowerScoringStart = poseFactory.of(11.3555, 9.1201, 180);
    public final Pose toFlowerScoring = poseFactory.of(46.7813, 121.6807, 90);
    public final Pose toFlowerScoringControl1 = poseFactory.of(16.8906, 81.2109, 0);
    public final Pose parkScoringside = poseFactory.of(9.8936, 121.1826, 90);


    Follower follower;

    public Poses(Follower follower) {
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

    public Path firstShoot() {
        return line(follower.pose(), firstShoot).constant(firstShoot);
    }

    public Path gardenCollect() {
        return line(follower.pose(), gardenCollect).tangent();
    }

    public Path secondShoot() {
        return curve(follower.pose(), secondShootStart, secondShootControl1, secondShootControl2, secondShoot).linear(secondShootStart, secondShoot);
    }

    public Path firstFlower() {
        return curve(follower.pose(), secondShoot, firstFlowerControl1, firstFlower).constant(firstFlower);
    }

    public Path thirdShoot() {
        return curve(follower.pose(), firstFlower, thirdShootControl1, thirdShoot).constant(thirdShoot);
    }

    public Path secondFlower() {
        return curve(follower.pose(), secondFlowerStart, secondFlowerControl1, secondFlowerControl2, secondFlower).linear(secondFlowerStart, secondFlower);
    }

    public Path fourthShoot() {
        return curve(follower.pose(), fourthShootStart, fourthShootControl1, fourthShoot).linear(fourthShootStart, fourthShoot);
    }

    public Path parkAuto() {
        return curve(follower.pose(), parkAutoStart, parkAutoControl1, parkAuto).linear(parkAutoStart, parkAuto);
    }

    private final Pose START = poseFactory.of(9.7, 131, -90);
    private final Pose path1 = poseFactory.of(10, 112, -90);

}