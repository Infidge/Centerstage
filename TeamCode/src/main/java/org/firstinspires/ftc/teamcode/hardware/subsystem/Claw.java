package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;
import org.firstinspires.ftc.teamcode.hardware.common.BreakBeam;

public class Claw {

	public final OptimisedServo rotation = new OptimisedServo();
	private final OptimisedServo pixelLeft = new OptimisedServo();
	private final OptimisedServo pixelRight = new OptimisedServo();

	private ClawStates.Rotation rotationState = ClawStates.Rotation.HORIZONTAL_TWO;
	private ClawStates.PixelLeft pixelLeftState = ClawStates.PixelLeft.OPEN;
	private ClawStates.PixelRight pixelRightState = ClawStates.PixelRight.OPEN;

	public void init(HardwareMap hwMap) {
		rotation.setName("clawRotation", hwMap);
		rotation.setPosition(rotationState.getPos(), true);

		pixelLeft.setName("clawPixelLeft", hwMap);
		pixelLeft.setPosition(pixelLeftState.getPos(), true);

		pixelRight.setName("clawPixelRight", hwMap);
		pixelRight.setPosition(pixelRightState.getPos(), true);
	}

	public void update() {
		rotation.setPosition(rotationState.getPos());
		pixelLeft.setPosition(pixelLeftState.getPos());
		pixelRight.setPosition(pixelRightState.getPos());
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

	public boolean pixelLeftIsOpen() {
		return pixelLeftState == ClawStates.PixelLeft.OPEN;
	}

	public boolean pixelRightIsOpen() {
		return pixelRightState == ClawStates.PixelRight.OPEN;
	}

}
