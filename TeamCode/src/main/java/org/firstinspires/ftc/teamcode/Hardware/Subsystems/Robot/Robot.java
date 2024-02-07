package org.firstinspires.ftc.teamcode.Hardware.Subsystems.Robot;

import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Claw.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.HorizontalExtension.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.Lift.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Subsystems.V4B.V4B;

public class Robot {
	public Drivetrain drivetrain = new Drivetrain();
	public HorizontalExtension extendo = new HorizontalExtension();
	public Intake intake = new Intake();
	public Lift lift = new Lift();
	public V4B v4b = new V4B();
	public Claw claw = new Claw();

	public enum robotStates {
		
	}
}
