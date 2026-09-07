package org.firstinspires.ftc.teamcode.PedroPathing;

import com.aaravlabs.safepedropathing.follower.Follower;
import com.aaravlabs.safepedropathing.follower.FollowerConstants;
import com.aaravlabs.safepedropathing.paths.PathConstraints;
import com.aaravlabs.
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
	public static FollowerConstants followerConstants = new FollowerConstants();

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

	public static Follower createFollower(HardwareMap hardwareMap) {
//		return new FollowerBuilder(followerConstants, hardwareMap)
//				.pathConstraints(pathConstraints)
//				.build();
	}
}
