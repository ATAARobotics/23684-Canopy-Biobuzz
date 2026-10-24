package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

/** Tunables, edited live from the Panels Configurables dashboard; read them every loop. */
@Configurable
public class TurretConfig {

	// "RED" or "BLUE".
	public static String alliance = "RED";

	private TurretConfig() {}

	/** Alliance letter for cluster names: 'R' for red, 'B' for blue. */
	public static char allianceLetter() {
		if (alliance != null && alliance.trim().toUpperCase().startsWith("B")) {
			return 'B';
		}
		return 'R';
	}
}
