package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.subsystem.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystem.V4B;

public class Robot {

	private static Robot instance = null;

	public final Drivetrain drivetrain;
	public final Intake intake;
	public final Lift lift;
	public final V4B v4b;
	public final Claw claw;

	private Robot()	{
		drivetrain = new Drivetrain();
		intake = new Intake();
		lift = new Lift();
		v4b = new V4B();
		claw = new Claw();
	}

	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}

	public void update() {
		intake.update();
	}

}
