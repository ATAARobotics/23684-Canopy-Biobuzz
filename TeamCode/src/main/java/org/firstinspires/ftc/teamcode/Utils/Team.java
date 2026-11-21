package org.firstinspires.ftc.teamcode.Utils;

/**
 * Which alliance the robot is on. OpModes return one of these from {@code getTeam()}, and the shared bases
 * use it to pick the starting pose, the vision alliance letter, and which HIVE cell the turret aims at.
 */
public enum Team {
	BLUE,
	RED,
	UNKNOWN;

	/** True only for the blue alliance; RED and UNKNOWN are not blue. */
	public boolean isBlue() {
		return this == BLUE;
	}

	/**
	 * Single-letter alliance tag shared across subsystems: {@code 'B'} for blue, {@code 'R'} for red.
	 * UNKNOWN reports {@code 'R'} so the robot defaults to the red side rather than failing closed.
	 */

	public char allianceLetter() {
		return isBlue() ? 'B' : 'R';
	}
}
