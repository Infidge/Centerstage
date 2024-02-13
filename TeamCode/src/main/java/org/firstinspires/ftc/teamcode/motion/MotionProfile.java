package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {

	private final double max_acceleration;
	private final double max_deceleration;
	private double max_velocity;

	private double acceleration_dt;
	private double cruise_dt;
	private double deceleration_dt;

	private double target;

	private final ElapsedTime elapsedTime = new ElapsedTime();

	MotionProfile(double max_acceleration, double max_deceleration, double max_velocity) {
		this.max_acceleration = max_acceleration;
		this.max_deceleration = max_deceleration;
		this.max_velocity = max_velocity;
	}

	public void setTarget(double target) {
		this.target = target;
	}

	public void init() {
		acceleration_dt = max_velocity / max_acceleration;

		double halfway_distance = target / 2;
		double acceleration_dist = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;

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
			double acceleration_dist = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;
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
