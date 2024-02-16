package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;
import org.firstinspires.ftc.teamcode.hardware.common.BreakBeam;

public class Claw {

	private final OptimisedServo rotation = new OptimisedServo();
	private final OptimisedServo pixelLeft = new OptimisedServo();
	private final OptimisedServo pixelRight = new OptimisedServo();

	private final BreakBeam leftBeam = new BreakBeam();
	private final BreakBeam rightBeam = new BreakBeam();

	private ClawStates.Rotation rotationState = ClawStates.Rotation.HORIZONTAL;
	private ClawStates.PixelLeft pixelLeftState = ClawStates.PixelLeft.OPEN;
	private ClawStates.PixelRight pixelRightState = ClawStates.PixelRight.OPEN;

	public void init(HardwareMap hwMap) {
		rotation.setName("clawRotation", hwMap);
		rotation.setPosition(rotationState.getPos());

		pixelLeft.setName("clawPixelLeft", hwMap);
		pixelLeft.setPosition(pixelLeftState.getPos());

		pixelRight.setName("clawPixelRight", hwMap);
		pixelRight.setPosition(pixelRightState.getPos());

		leftBeam.setName("clawLeftBeam", hwMap);
		rightBeam.setName("clawRightBeam", hwMap);
	}

	public void update() {
		rotation.setPosition(rotationState.getPos());
		pixelLeft.setPosition(pixelLeftState.getPos());
		pixelRight.setPosition(pixelRightState.getPos());

		if (leftBeam.isBroken()) {
			pixelLeftState = ClawStates.PixelLeft.CLOSED;
		}

		if (rightBeam.isBroken()) {
			pixelRightState = ClawStates.PixelRight.OPEN;
		}
	}

	public void rotateHorizontally() {
		rotationState = ClawStates.Rotation.HORIZONTAL;
	}

	public void rotateVertically() {
		rotationState = ClawStates.Rotation.VERTICAL;
	}

	public void pixelLeftClose() {
		pixelLeftState = ClawStates.PixelLeft.CLOSED;
	}

	public void pixelLeftOpen() {
		pixelLeftState = ClawStates.PixelLeft.OPEN;
	}

	public void pixelRightClose() {
		pixelRightState = ClawStates.PixelRight.CLOSED;
	}

	public void pixelRightOpen() {
		pixelRightState = ClawStates.PixelRight.OPEN;
	}

}
