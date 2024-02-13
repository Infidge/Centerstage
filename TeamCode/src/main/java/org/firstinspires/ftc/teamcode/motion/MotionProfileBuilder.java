package org.firstinspires.ftc.teamcode.motion;

public class MotionProfileBuilder {

	private double max_acceleration;
	private double max_deceleration;
	private double max_velocity;

	public MotionProfileBuilder setMaxAcceleration(double max_acceleration) {
		this.max_acceleration = max_acceleration;
		return this;
	}

	public MotionProfileBuilder setMaxDeceleration(double max_deceleration) {
		this.max_deceleration = max_deceleration;
		return this;
	}

	public MotionProfileBuilder setMaxVelocity(double max_velocity) {
		this.max_velocity = max_velocity;
		return this;
	}

	public MotionProfie build() {
		return new MotionProfie(max_acceleration, max_deceleration, max_velocity);
	}

}