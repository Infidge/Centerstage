package org.firstinspires.ftc.teamcode.Hardware.Subsystems.Claw;

import org.firstinspires.ftc.teamcode.Hardware.Optimised_Hardware.Optimised_Servo;
import org.firstinspires.ftc.teamcode.Utils.BreakBeams;

public class Claw
{
	Optimised_Servo leftClaw = new Optimised_Servo();
	Optimised_Servo rightClaw = new Optimised_Servo();
	Optimised_Servo rotation = new Optimised_Servo();

	BreakBeams leftBeam = new BreakBeams();
	BreakBeams rightBeam = new BreakBeams();

	LeftCLaw leftClawState = LeftCLaw.CLOSED;
	RightClaw rightClawState = RightClaw.CLOSED;
	Rotate rotationState = Rotate.UPRIGHT;

	public enum LeftCLaw
	{
		CLOSED,
		OPEN
	}

	public enum RightClaw
	{
		CLOSED,
		OPEN
	}

	public enum Rotate
	{
		UPRIGHT,
		UPSIDE_DOWN
	}

	public void autoClose()
	{
		if (leftBeam.isBroken())
			leftClawState = LeftCLaw.CLOSED;
		if (rightBeam.isBroken())
			rightClawState = RightClaw.CLOSED; 

	}
}
