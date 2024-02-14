package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {

	private double max_acceleration;
	private double max_deceleration;
	private double max_velocity;

	private double acceleration_dt;
	private double cruise_dt;
	private double deceleration_dt;

	private double acceleration_dist;

	private double target;

	private final ElapsedTime elapsedTime = new ElapsedTime();

	public MotionProfile() {}

	public void setTarget(double target) {
		this.target = target;
	}

	public void init(double max_acc, double max_dec, double max_vel) {
		max_acceleration = max_acc;
		max_deceleration = max_dec;
		max_velocity = max_vel;

		acceleration_dt = max_velocity / max_acceleration;

		double halfway_distance = target / 2;
		acceleration_dist = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;

		if (acceleration_dist > halfway_distance) {
			acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
			acceleration_dist = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;
		}

		max_velocity = max_acceleration * acceleration_dt;

		double cruise_dist = target - 2 * acceleration_dist;
		cruise_dt = cruise_dist / max_velocity;

		deceleration_dt = acceleration_dt;

		elapsedTime.reset();
	}

	public double update() {
		double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
		double dt = elapsedTime.seconds();

		if (dt > entire_dt) {
			// finished the motion profile
			return target;

		} else if (dt < acceleration_dt) {
			// accelerating
			return 0.5 * max_acceleration * dt * dt;

		} else if (dt < acceleration_dt + cruise_dt) {
			// cruising
			double cruise_current_dt = dt - acceleration_dt;

			return acceleration_dist + max_velocity * cruise_current_dt;

		} else {
			// decelerating
			double acceleration_dist = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;
			double cruise_dist = max_velocity * cruise_dt;
			double deceleration_current_time = dt - acceleration_dt - cruise_dt;

			return acceleration_dist + cruise_dist + max_velocity * deceleration_current_time - 0.5 * max_acceleration * deceleration_current_time * deceleration_current_time;
		}

	}

}
