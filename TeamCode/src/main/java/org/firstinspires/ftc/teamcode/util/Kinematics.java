package org.firstinspires.ftc.teamcode.util;

public class Kinematics {

	public static double distFromAccelAndTime(double accel, double time) {
		return 0.5 * accel * time * time;
	}

	public static double distFromVelAccelAndTime(double vel, double accel, double time) {
		return vel * time + 0.5 * accel * time * time;
	}

	public static double timeFromDistAndAcc(double dist, double accel) {
		return Math.sqrt(2.0 * dist / accel);
	}

}
