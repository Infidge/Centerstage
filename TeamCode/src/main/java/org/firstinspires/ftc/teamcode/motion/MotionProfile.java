package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
	private double deceleration_dist;

	private double target_dist;

	private final ElapsedTime elapsedTime = new ElapsedTime();

	public MotionProfile(double max_acc, double max_dec, double max_vel) {
		setConstraints(max_acc, max_dec, max_vel);
	}

	public void setConstraints(double max_acc, double max_dec, double max_vel) {
		max_acceleration = max_acc;
		max_deceleration = max_dec;
		max_velocity = max_vel;
	}

	public void setDistance(double dist) {
		target_dist = dist;
	}

	public void start() {
//		acceleration_dt = max_velocity / max_acceleration;
//		double deceleration_dt = max_velocity / max_deceleration;
//
//		acceleration_dist = Kinematics.distFromAccelAndTime(max_acceleration, acceleration_dt);
//		double deceleration_dist = Kinematics.distFromAccelAndTime(max_deceleration, deceleration_dt);
//
//		if (acceleration_dist + deceleration_dist > target_dist) {
//			acceleration_dist = target_dist * acceleration_dist / (acceleration_dist + deceleration_dist);
//			deceleration_dist = target_dist * deceleration_dist / (acceleration_dist + deceleration_dist);
//
//			acceleration_dt = Kinematics.timeFromDistAndAcc(acceleration_dist, max_acceleration);
//			deceleration_dt = Kinematics.timeFromDistAndAcc(deceleration_dist, max_deceleration);
//		}
//
//		max_velocity = max_acceleration * acceleration_dt;
//
//		cruise_dist = target_dist - acceleration_dist - deceleration_dist;
//		cruise_dt = cruise_dist / max_velocity;
//
//		total_dt = acceleration_dt + cruise_dt + deceleration_dt;
//
//		elapsedTime.reset();

		acceleration_dt = max_velocity / max_acceleration;
		deceleration_dt = max_velocity / max_deceleration;

		acceleration_dist = Kinematics.distFromAccelAndTime(max_acceleration, acceleration_dt);
		deceleration_dist = Kinematics.distFromAccelAndTime(max_deceleration, deceleration_dt);

		if (acceleration_dist + deceleration_dist > target_dist) {
			acceleration_dist = target_dist * acceleration_dist / (acceleration_dist + deceleration_dist);
			deceleration_dist = target_dist * deceleration_dist / (acceleration_dist + deceleration_dist);

			acceleration_dt = Kinematics.timeFromDistAndAcc(acceleration_dist, max_acceleration);
			deceleration_dt = Kinematics.timeFromDistAndAcc(deceleration_dist, max_deceleration);
		}

//		if (acceleration_dist > target_dist / 2) {
//			acceleration_dt = Kinematics.timeFromDistAndAcc(target_dist / 2, max_acceleration);
//			acceleration_dist = target_dist / 2;
//		}

		max_velocity = Math.min(max_acceleration * acceleration_dt, max_deceleration * deceleration_dt);

//		deceleration_dt = acceleration_dt;

		cruise_dist = target_dist - acceleration_dist - deceleration_dist;
		if (cruise_dist < 0) cruise_dist = 0;

		cruise_dt = cruise_dist / max_velocity;

		total_dt = acceleration_dt + cruise_dt + deceleration_dt;

		elapsedTime.reset();
	}

	public double getInstantPosition(Telemetry telemetry) {
		double dt = elapsedTime.seconds();

		telemetry.addData("acceleration_dt", acceleration_dt);
		telemetry.addData("cruise_dt", cruise_dt);
		telemetry.addData("deceleration_dt", deceleration_dt);

		telemetry.addData("acceleration_dist", acceleration_dist);
		telemetry.addData("cruise_dist", cruise_dist);
		telemetry.addData("deceleration_dist", deceleration_dist);

		telemetry.addData("acceleration_start", 0);
		telemetry.addData("cruise_start", acceleration_dist);
		telemetry.addData("deceleration_start", acceleration_dist + cruise_dist);

		telemetry.addData("dt", dt);
		telemetry.addData("entire_dt", total_dt);

		if (dt >= total_dt) {
			// finished the motion profile
			return target_dist;

		} else if (dt < acceleration_dt) {
			// accelerating
			return Kinematics.distFromAccelAndTime(max_acceleration, dt);

		} else if (dt < acceleration_dt + cruise_dt) {
			// cruising
			double cruise_current_dt = dt - acceleration_dt;
			double cruise_current_dist = max_velocity * cruise_current_dt;

			return acceleration_dist + cruise_current_dist;

		} else {
			// decelerating
			double deceleration_current_dt = dt - acceleration_dt - cruise_dt;
			double deceleration_current_dist = Kinematics.distFromVelAccelAndTime(max_velocity, -max_deceleration, deceleration_current_dt);

			return acceleration_dist + cruise_dist + deceleration_current_dist;
		}

//		if (dt > total_dt) {
//			return target_dist;
//		} else if (dt < acceleration_dt) {
//			return 0.5 * max_acceleration * dt * dt;
//		} else if (dt < deceleration_time) {
//			acceleration_dist = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;
//			double cruise_current_dt = dt - acceleration_dt;
//
//			return acceleration_dist + max_velocity * cruise_current_dt;
//		} else {
//			acceleration_dist = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;
//			double cruise_distance = max_velocity * cruise_dt;
//			double deceleration_time = dt - this.deceleration_time;
//
//			return acceleration_dist + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time * deceleration_time;
//		}
	}

}
