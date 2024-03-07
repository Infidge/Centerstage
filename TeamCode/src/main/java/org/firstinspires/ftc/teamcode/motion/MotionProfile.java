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

	private boolean finished = true;

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

	public void start(Telemetry telemetry) {
		if (target_dist < 0.01)
			return;

		acceleration_dt = max_velocity / max_acceleration;
		deceleration_dt = max_velocity / max_deceleration;

		acceleration_dist = Kinematics.distFromAccelAndTime(max_acceleration, acceleration_dt);
		deceleration_dist = Kinematics.distFromAccelAndTime(max_deceleration, deceleration_dt);

		if (acceleration_dist + deceleration_dist > target_dist) {
			acceleration_dist = target_dist * acceleration_dist / (acceleration_dist + deceleration_dist);
			deceleration_dist = target_dist * deceleration_dist / (acceleration_dist + deceleration_dist);

			acceleration_dt = Kinematics.timeFromDistAndAcc(acceleration_dist, max_acceleration);

			max_velocity = max_acceleration * acceleration_dt;

//			double equation_a = 0.5 * max_deceleration;
//			double equation_b = max_velocity;
//			double equation_c = -deceleration_dist;
//
//			double equation_delta = equation_b * equation_b - 4 * equation_a * equation_c;
//
//			double x1 = (-equation_b + Math.sqrt(equation_delta)) / (2 * equation_a);
//			double x2 = (-equation_b - Math.sqrt(equation_delta)) / (2 * equation_a);

//			telemetry.addData("equation_x1", x1);
//			telemetry.addData("equation_x2", x2);
			deceleration_dt = Kinematics.timeFromDistAndAcc(deceleration_dist, max_deceleration);
		}

//		max_velocity = Math.min(max_acceleration * acceleration_dt, max_deceleration * deceleration_dt);

		cruise_dist = target_dist - acceleration_dist - deceleration_dist;
		if (cruise_dist < 0) cruise_dist = 0;

		cruise_dt = cruise_dist / max_velocity;

		total_dt = acceleration_dt + cruise_dt + deceleration_dt;

		finished = false;

		elapsedTime.reset();
	}

	public double getInstantPosition() {
		double dt = elapsedTime.seconds();

		if (dt >= total_dt) {
			// finished the motion profile
			finished = true;
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
	}

	public boolean isFinished() {
		return finished;
	}

	public double getDistance() {
		return target_dist;
	}

}
