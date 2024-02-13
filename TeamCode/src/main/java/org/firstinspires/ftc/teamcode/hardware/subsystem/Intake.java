package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.optimised.OptimisedMotor;
import org.firstinspires.ftc.teamcode.hardware.optimised.OptimisedServo;
import org.firstinspires.ftc.teamcode.util.IRBreakBeam;

public class Intake {

	private OptimisedServo angle;
	private OptimisedMotor horizontalExtension;
	private OptimisedMotor spinners;
	private IRBreakBeam leftBeam;
	private IRBreakBeam rightBeam;

	private IntakeStates.Angle angleState = IntakeStates.Angle.TRANSFER;
	private IntakeStates.Extension extensionState = IntakeStates.Extension.IN;
	private IntakeStates.Spinners spinnersState = IntakeStates.Spinners.CHILL;

	public void init(HardwareMap hwMap) {
		horizontalExtension.setName("intakeHorizontalExtension", hwMap);
		horizontalExtension.setDirection(DcMotorSimple.Direction.FORWARD);
		horizontalExtension.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		horizontalExtension.setPower(0.0);

		spinners.setName("intakeSpinners", hwMap);
		spinners.setDirection(DcMotorSimple.Direction.FORWARD);
		spinners.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		spinners.setPower(spinnersState.getPower());

		angle.setName("intakeAngle", hwMap);
		angle.setPosition(angleState.getPos());

		leftBeam.setName("intakeLeftBeam", hwMap);
		rightBeam.setName("intakeRightBeam", hwMap);
	}

	public void spinInwards() {
		spinnersState = IntakeStates.Spinners.IN;
	}

	public void spinOutwards() {
		spinnersState = IntakeStates.Spinners.OUT;
	}

	public void spinStop() {
		spinnersState = IntakeStates.Spinners.CHILL;
	}

	public void angleToCollect() {
		angleState = IntakeStates.Angle.COLLECT;
	}

	public void angleToTransfer() {
		angleState = IntakeStates.Angle.TRANSFER;
	}

	public void update() {
		if (leftBeam.isBroken() && rightBeam.isBroken() && angleState == IntakeStates.Angle.COLLECT) {
			angleState = IntakeStates.Angle.TRANSFER;
		}

		spinners.setPower(spinnersState.getPower());
		angle.setPosition(angleState.getPos());
	}

}
