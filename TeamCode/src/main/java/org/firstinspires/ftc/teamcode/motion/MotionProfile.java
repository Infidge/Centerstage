package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Kinematics;

public class MotionProfile {

	private double max_acceleration;
	private double max_deceleration;
	private double max_velocity;

	private double acceleration_dt;
	private double cruise_dt;
	private double deceleration_dt;
	private double total_dt;

	private double acceleration_dist;
	private double cruise_dist;

	private double target_dist;

	private final ElapsedTime elapsedTime = new ElapsedTime();

	public MotionProfile(double max_acc, double max_dec, double max_vel) {
		max_acceleration = max_acc;
		max_deceleration = max_dec;
		max_velocity = max_vel;
	}

	public void init() {
		acceleration_dt = max_velocity / max_acceleration;

		double half_dist = target_dist / 2;
		acceleration_dist = Kinematics.distFromAccelAndTime(max_acceleration, acceleration_dt);

		if (acceleration_dist > half_dist) {
			acceleration_dt = Kinematics.timeFromDistAndAcc(half_dist, max_acceleration);
			acceleration_dist = Kinematics.distFromAccelAndTime(max_acceleration, acceleration_dt);
		}

		max_velocity = max_acceleration * acceleration_dt;

		cruise_dist = target_dist - 2 * acceleration_dist; // TODO
		cruise_dt = cruise_dist / max_velocity;

		deceleration_dt = acceleration_dt;

		total_dt = acceleration_dt + cruise_dt + deceleration_dt;
	}

	public void start(double target) {
		target_dist = target;
		elapsedTime.reset();
	}

	public double update() {
		double dt = elapsedTime.seconds();

		if (dt >= total_dt) { // finished the motion profile
			return target_dist;

		} else if (dt < acceleration_dt) { // accelerating
			return Kinematics.distFromAccelAndTime(max_acceleration, dt);

		} else if (dt < acceleration_dt + cruise_dt) { // cruising
			double cruise_current_dt = dt - acceleration_dt;
			double cruise_current_dist = max_velocity * cruise_current_dt;

			return acceleration_dist + cruise_current_dist;

		} else { // decelerating
			double deceleration_current_dt = dt - acceleration_dt - cruise_dt;

			return acceleration_dist + cruise_dist + Kinematics.distFromVelAccelAndTime(-max_acceleration, max_velocity, deceleration_current_dt);
		}
	}

}
