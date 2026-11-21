package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.aaravlabs.safepedropathing.api.PoseFactory;
import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Config.FieldMap;
import org.firstinspires.ftc.teamcode.Utils.Team;

/**
 * Field poses for the autonomous routes, in Pedro inches and degrees.
 *
 * <p>Coordinates are authored once for the red half of the field. {@link #onSide} hands back the same poses
 * rotated 180° about the field's center for the blue half, so a blue auto is the red auto flipped rather
 * than a second set of hand-tuned numbers.
 */
public class Poses {

	public static Pose finalPose = new Pose(0, 0, 0);

	private static final PoseFactory DEGREES = PoseFactory.degrees();

	// Audience half.
	public final Pose startAudience;
	public final Pose audienceLeave;
	public final Pose audienceFlowerIntake;
	public final Pose audienceFlowerLeave;
	public final Pose toGarden;
	public final Pose pickupGarden;
	public final Pose thenShoot;
	public final Pose audiencePark;
	public final Pose audienceParkControl;

	// Scoring half.
	public final Pose startScoring;
	public final Pose scoringLeave;
	public final Pose scoringFlowerIntake;
	public final Pose scoringFlowerLeave;
	public final Pose scoringPark;
	public final Pose scoringParkControl1;
	public final Pose scoringParkControl2;

	// depart -> shoot -> garden -> shoot -> flower -> shoot -> flower -> shoot -> park
	public final Pose firstShoot;
	public final Pose secondShootStart;
	public final Pose secondShoot;
	public final Pose thirdShoot;
	public final Pose fourthShootStart;
	public final Pose fourthShoot;

	public final Pose secondShootControl1;
	public final Pose secondShootControl2;
	public final Pose firstFlowerControl1;
	public final Pose thirdShootControl1;
	public final Pose secondFlowerControl1;
	public final Pose secondFlowerControl2;
	public final Pose fourthShootControl1;
	public final Pose parkControl1;

	public final Pose parkingZoneStart;
	public final Pose parkingZoneAlliancePush;
	public final Pose toFlowerScoringControl1;

	// Solo route starts on the audience side, near the audience flower.
	public final Pose startSolo;

	// True once the poses are mirrored, so the same field coordinates serve both alliances.
	private final boolean mirrored;

	/** Poses for one alliance's half of the field. Blue is the red half rotated 180° about the field center. */
	public static Poses onSide(Follower follower, Team team) {
		return new Poses(follower, team == Team.BLUE);
	}

	private Poses(Follower follower, boolean mirrored) {
		this.mirrored = mirrored;

		startAudience = pose(55, 9, 90);
		audienceLeave = pose(55, 18, 90);
		audienceFlowerIntake = pose(12.7, 47.5, 180);
		audienceFlowerLeave = pose(29.5, 30, 180);
		toGarden = pose(23.5, 9.5, 180);
		pickupGarden = pose(7.5, 9.8, 180);
		thenShoot = pose(30, 30, 41.9168);
		audiencePark = pose(16.5, 96.75, 106.4792);
		audienceParkControl = pose(3, 38.9097, 0);

		startScoring = pose(60.3538, 133.5359, 270);
		scoringLeave = pose(60.3538, 116.909, 270);
		scoringFlowerIntake = pose(46.2697, 127.238, 90);
		scoringFlowerLeave = pose(51.357346875, 106.5348046875, 90);
		scoringPark = pose(11.3745, 119.344, 90);
		scoringParkControl1 = pose(52.0623, 99.3749, 0);
		scoringParkControl2 = pose(32.0284, 126.9064, 0);

		firstShoot = pose(56, 10, 180);
		secondShootStart = pose(10, 10, 180);
		secondShoot = pose(53, 124, 90);
		thirdShoot = pose(53, 124, 90);
		fourthShootStart = pose(12, 47, 180);
		fourthShoot = pose(56, 10, 90);

		secondShootControl1 = pose(24, 54, 0);
		secondShootControl2 = pose(20, 122, 0);
		firstFlowerControl1 = pose(46, 119, 0);
		thirdShootControl1 = pose(46, 119, 0);
		secondFlowerControl1 = pose(16, 97, 0);
		secondFlowerControl2 = pose(40, 47, 0);
		fourthShootControl1 = pose(58, 36, 0);
		parkControl1 = pose(32, 40, 0);

		parkingZoneStart = pose(7.8945, 59.4873, 90);
		parkingZoneAlliancePush = pose(7.8945, 82.0361, 270);
		toFlowerScoringControl1 = pose(16.8906, 81.2109, 0);

		startSolo = pose(8, 49, 0);
	}

	private Pose pose(double x, double y, double headingDegrees) {
		Pose pose = DEGREES.of(x, y, headingDegrees);
		return mirrored ? mirror(pose) : pose;
	}

	/** Mirrors a pose through the field's center: X, Y, and heading all flip. */
	public static Pose mirror(Pose pose) {
		return pose.withX(FieldMap.mirror(pose.x()))
				.withY(FieldMap.mirror(pose.y()))
				.withHeading(pose.heading() + Math.PI);
	}
}
