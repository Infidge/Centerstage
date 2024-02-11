package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import org.firstinspires.ftc.teamcode.Hardware.Optimised_Hardware.OptimisedServo;
import org.firstinspires.ftc.teamcode.Utils.IRBreakBeam;

public class Claw {

	private final OptimisedServo leftClaw = new OptimisedServo();
	private final OptimisedServo rightClaw = new OptimisedServo();
	private final OptimisedServo rotation = new OptimisedServo();

	private final IRBreakBeam leftBeam = new IRBreakBeam();
	private final IRBreakBeam rightBeam = new IRBreakBeam();

	private LeftCLaw leftClawState = LeftCLaw.CLOSED;
	private RightClaw rightClawState = RightClaw.CLOSED;
	private RotateClaw rotationState = RotateClaw.UPRIGHT;

	private enum LeftCLaw {
		CLOSED,
		OPEN
	}

	private enum RightClaw {
		CLOSED,
		OPEN
	}

	private enum RotateClaw {
		UPRIGHT,
		UPSIDE_DOWN
	}

	public void update() {
		if (leftBeam.isBroken()) {
			leftClawState = LeftCLaw.CLOSED;
		}

		if (rightBeam.isBroken()) {
			rightClawState = RightClaw.CLOSED;
		}
	}

}
