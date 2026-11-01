package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.api.PoseFactory;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;

public class Poses {


    //TODO: Glean out repeating poses. we only need one per location.

    public final PoseFactory redposeFactory = PoseFactory.degrees();

    public final Pose redStartAudience = redposeFactory.of(55, 9, 90);
    public final Pose redAudienceLeave = redposeFactory.of(55, 18, 90);
    public final Pose redAudienceFlowerIntake = redposeFactory.of(12.7, 47.5, 180);
    public final Pose redAudienceFlowerLeave = redposeFactory.of(29.5, 30, 180);
    public final Pose redToGarden = redposeFactory.of(23.5, 9.5, 180);
    public final Pose redPickupGarden = redposeFactory.of(7.5, 9.8, 180);
    public final Pose redThenShoot = redposeFactory.of(30, 30, 41.9168);
    public final Pose redAudiencePark = redposeFactory.of(11.5, 92.5382, 106.4792);


    public final Pose redStartScoring = redposeFactory.of(60.3538, 133.5359, 270);
    public final Pose redScoringLeave = redposeFactory.of(60.3538, 116.909, 270);
    public final Pose redScoringFlowerIntake = redposeFactory.of(46.2697, 127.238, 90);
    public final Pose redScoringFlowerLeave = redposeFactory.of(51.357346875, 106.5348046875, 90);
    public final Pose redScoringPark = redposeFactory.of(8.016, 122.9, 90);

    public final Pose firstShoot = redposeFactory.of(56, 10, 180);
    public final Pose secondShootStart = redposeFactory.of(10, 10, 180);
    public final Pose secondShoot = redposeFactory.of(53, 124, 90);
    public final Pose thirdShoot = redposeFactory.of(53, 124, 90);
    public final Pose fourthShootStart = redposeFactory.of(12, 47, 180);
    public final Pose fourthShoot = redposeFactory.of(56, 10, 90);
    // depart -> shoot -> garden -> shoot -> flower -> shoot -> flower -> shoot -> park

    public final Pose secondShootControl1 = redposeFactory.of(24, 54, 0);
    public final Pose secondShootControl2 = redposeFactory.of(20, 122, 0);
    public final Pose firstFlowerControl1 = redposeFactory.of(46, 119, 0);
    public final Pose thirdShootControl1 = redposeFactory.of(46, 119, 0);
    public final Pose secondFlowerControl1 = redposeFactory.of(16, 97, 0);
    public final Pose secondFlowerControl2 = redposeFactory.of(40, 47, 0);
    public final Pose fourthShootControl1 = redposeFactory.of(58, 36, 0);
    public final Pose parkAutoControl1 = redposeFactory.of(32, 40, 0);


    public final Pose redParkingZoneStart = redposeFactory.of(7.8945, 59.4873, 90);
    public final Pose redParkingZoneAlliancePush = redposeFactory.of(7.8945, 82.0361, 270);
    public final Pose toFlowerScoringControl1 = redposeFactory.of(16.8906, 81.2109, 0);

    PoseFactory blueposeFactory = redposeFactory.rotateAround(72,72, 180);

    public final Pose blueStartAudience = blueposeFactory.of(55, 9, 90);
    public final Pose blueAudienceLeave = blueposeFactory.of(55, 18, 90);
    public final Pose blueAudienceFlowerIntake = blueposeFactory.of(12.7, 47.5, 180);
    public final Pose blueAudienceFlowerLeave = blueposeFactory.of(29.5, 30, 180);
    public final Pose blueToGarden = blueposeFactory.of(23.5, 9.5, 180);
    public final Pose bluePickupGarden = blueposeFactory.of(7.5, 9.8, 180);
    public final Pose blueThenShoot = blueposeFactory.of(30, 30, 41.9168);
    public final Pose blueAudiencePark = blueposeFactory.of(11.5, 92.5382, 106.4792);


    public final Pose blueStartScoring = blueposeFactory.of(60.3538, 133.5359, 270);
    public final Pose blueScoringLeave = blueposeFactory.of(60.3538, 116.909, 270);
    public final Pose bleuScoringFlowerIntake = blueposeFactory.of(46.2697, 127.238, 90);
    public final Pose blueScoringFlowerLeave = blueposeFactory.of(51.357346875, 106.5348046875, 90);
    public final Pose blueScoringPark = blueposeFactory.of(8.016, 122.9, 90);

    public final Pose bluefirstShoot = blueposeFactory.of(56, 10, 180);
    public final Pose bluesecondShootStart = blueposeFactory.of(10, 10, 180);
    public final Pose bluesecondShoot = blueposeFactory.of(53, 124, 90);
    public final Pose bluethirdShoot = blueposeFactory.of(53, 124, 90);
    public final Pose bluefourthShootStart = blueposeFactory.of(12, 47, 180);
    public final Pose bluefourthShoot = blueposeFactory.of(56, 10, 90);
    // depart -> shoot -> garden -> shoot -> flower -> shoot -> flower -> shoot -> park

    public final Pose bluesecondShootControl1 = blueposeFactory.of(24, 54, 0);
    public final Pose bluesecondShootControl2 = blueposeFactory.of(20, 122, 0);
    public final Pose bluefirstFlowerControl1 = blueposeFactory.of(46, 119, 0);
    public final Pose bluethirdShootControl1 = blueposeFactory.of(46, 119, 0);
    public final Pose bluesecondFlowerControl1 = blueposeFactory.of(16, 97, 0);
    public final Pose bluesecondFlowerControl2 = blueposeFactory.of(40, 47, 0);
    public final Pose bluefourthShootControl1 = blueposeFactory.of(58, 36, 0);
    public final Pose blueparkAutoControl1 = blueposeFactory.of(32, 40, 0);


    public final Pose blueParkingZoneStart = blueposeFactory.of(7.8945, 59.4873, 90);
    public final Pose blueParkingZoneAlliancePush = blueposeFactory.of(7.8945, 82.0361, 270);
    public final Pose bluetoFlowerScoringControl1 = blueposeFactory.of(16.8906, 81.2109, 0);



    Follower follower;

    public Poses(Follower follower) {
        this.follower = follower;
    }
}