package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.api.PoseFactory;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;

public class Poses {

	public static Pose finalPose = new Pose(0, 0, 0);

	public final PoseFactory redPoseFactory = PoseFactory.degrees();

	public final Pose redStartAudience = redPoseFactory.of(55, 9, 90);
	public final Pose redAudienceLeave = redPoseFactory.of(55, 18, 90);
	public final Pose redAudienceFlowerIntake = redPoseFactory.of(12.7, 47.5, 180);
	public final Pose redAudienceFlowerLeave = redPoseFactory.of(29.5, 30, 180);
	public final Pose redToGarden = redPoseFactory.of(23.5, 9.5, 180);
	public final Pose redPickupGarden = redPoseFactory.of(7.5, 9.8, 180);
	public final Pose redThenShoot = redPoseFactory.of(30, 30, 41.9168);
	public final Pose redAudiencePark = redPoseFactory.of(16.5, 97.5382, 106.4792);
	public final Pose redAudienceParkControl = redPoseFactory.of(3, 38.9097, 0);

	public final Pose redStartScoring = redPoseFactory.of(60.3538, 133.5359, 270);
	public final Pose redScoringLeave = redPoseFactory.of(60.3538, 116.909, 270);
	public final Pose redScoringFlowerIntake = redPoseFactory.of(46.2697, 127.238, 90);
	public final Pose redScoringFlowerLeave = redPoseFactory.of(51.357346875, 106.5348046875, 90);
	public final Pose redScoringPark = redPoseFactory.of(9.9916, 121.5171, 90);
	public final Pose redScoringParkControl1 = redPoseFactory.of(52.0623, 99.3749, 0);
	public final Pose redScoringParkControl2 = redPoseFactory.of(32.0284, 126.9064, 0);

	public final Pose redFirstShoot = redPoseFactory.of(56, 10, 180);
	public final Pose redSecondShootStart = redPoseFactory.of(10, 10, 180);
	public final Pose redSecondShoot = redPoseFactory.of(53, 124, 90);
	public final Pose redThirdShoot = redPoseFactory.of(53, 124, 90);
	public final Pose redFourthShootStart = redPoseFactory.of(12, 47, 180);
	public final Pose redFourthShoot = redPoseFactory.of(56, 10, 90);
	// depart -> shoot -> garden -> shoot -> flower -> shoot -> flower -> shoot -> park

	public final Pose redSecondShootControl1 = redPoseFactory.of(24, 54, 0);
	public final Pose redSecondShootControl2 = redPoseFactory.of(20, 122, 0);
	public final Pose redFirstFlowerControl1 = redPoseFactory.of(46, 119, 0);
	public final Pose redThirdShootControl1 = redPoseFactory.of(46, 119, 0);
	public final Pose redSecondFlowerControl1 = redPoseFactory.of(16, 97, 0);
	public final Pose redSecondFlowerControl2 = redPoseFactory.of(40, 47, 0);
	public final Pose redFourthShootControl1 = redPoseFactory.of(58, 36, 0);
	public final Pose redParkControl1 = redPoseFactory.of(32, 40, 0);

	public final Pose redParkingZoneStart = redPoseFactory.of(7.8945, 59.4873, 90);
	public final Pose redParkingZoneAlliancePush = redPoseFactory.of(7.8945, 82.0361, 270);
	public final Pose redToFlowerScoringControl1 = redPoseFactory.of(16.8906, 81.2109, 0);

	// TODO: Test Blue please :P

	PoseFactory bluePoseFactory = PoseFactory.degrees();

	public final Pose blueStartAudience = bluePoseFactory.of(82.15, 9, 90);
	public final Pose blueAudienceLeave = bluePoseFactory.of(82.15, 23.12, 90);
	public final Pose blueAudienceFlowerIntake = bluePoseFactory.of(94.75, 15.05, 270);
	public final Pose blueAudienceFlowerLeave = bluePoseFactory.of(94.75, 22.5, 270);
	public final Pose blueToGarden = bluePoseFactory.of(109.6, 135, 0);
	public final Pose bluePickupGarden = bluePoseFactory.of(129.2, 135, 0);
	public final Pose blueThenShoot = bluePoseFactory.of(119, 119, 41.9168);
	public final Pose blueAudiencePark = bluePoseFactory.of(129, 21, 106.4792);

	public final Pose blueStartScoring = bluePoseFactory.of(85, 113.3, 270);
	public final Pose blueScoringLeave = bluePoseFactory.of(85, 118.9, 270);
	public final Pose blueScoringFlowerIntake = bluePoseFactory.of(126.1, 94.3, 0);
	public final Pose blueScoringFlowerLeave = bluePoseFactory.of(119.5, 94.3, 0);
	public final Pose blueScoringPark = bluePoseFactory.of(133.1, 52.1, 90);

	//    public final Pose blueFirstShoot = bluePoseFactory.of(56, 10, 180);
	//    public final Pose blueSecondShootStart = bluePoseFactory.of(10, 10, 180);
	//    public final Pose blueSecondShoot = bluePoseFactory.of(53, 124, 90);
	//    public final Pose blueThirdShoot = bluePoseFactory.of(53, 124, 90);
	//    public final Pose blueFourthShootStart = bluePoseFactory.of(12, 47, 180);
	//    public final Pose blueFourthShoot = bluePoseFactory.of(56, 10, 90);
	// depart -> shoot -> garden -> shoot -> flower -> shoot -> flower -> shoot -> park

	//    public final Pose blueSecondShootControl1 = bluePoseFactory.of(24, 54, 0);
	//    public final Pose blueSecondShootControl2 = bluePoseFactory.of(20, 122, 0);
	//    public final Pose blueFirstFlowerControl1 = bluePoseFactory.of(46, 119, 0);
	//    public final Pose blueThirdShootControl1 = bluePoseFactory.of(46, 119, 0);
	//    public final Pose blueSecondFlowerControl1 = bluePoseFactory.of(16, 97, 0);
	//    public final Pose blueSecondFlowerControl2 = bluePoseFactory.of(40, 47, 0);
	//    public final Pose blueFourthShootControl1 = bluePoseFactory.of(58, 36, 0);
	//    public final Pose blueParkAutoControl1 = bluePoseFactory.of(32, 40, 0);

	public final Pose blueParkingZoneStart = bluePoseFactory.of(133, 71.6, 90);
	public final Pose blueParkingZoneAlliancePush = bluePoseFactory.of(133, 61, 270);
	public final Pose blueToFlowerScoringControl1 = bluePoseFactory.of(16.8906, 81.2109, 0);

	Follower follower;

	public Poses(Follower follower) {
		this.follower = follower;
	}
}
