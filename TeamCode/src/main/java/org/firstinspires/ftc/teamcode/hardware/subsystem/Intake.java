package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotor;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;
import org.firstinspires.ftc.teamcode.hardware.common.IRBreakBeam;

public class Intake {

	private OptimisedServo angle;
	private OptimisedMotor horizontalExtension;
	private LimitSwitch limitSwitch;
	private OptimisedMotor spinners;
	private IRBreakBeam leftBeam;
	private IRBreakBeam rightBeam;

	private IntakeStates.Angle angleState = IntakeStates.Angle.TRANSFER;
	private IntakeStates.Extension extensionState = IntakeStates.Extension.IN;
	private IntakeStates.Spinners spinnersState = IntakeStates.Spinners.CHILL;

	public void init(HardwareMap hwMap) {
		angle.setName("intakeAngle", hwMap);
		angle.setPosition(angleState.getPos());

		horizontalExtension.setName("intakeHorizontalExtension", hwMap);
		horizontalExtension.setDirection(DcMotorSimple.Direction.FORWARD);
		horizontalExtension.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		horizontalExtension.setPower(0.0);

		limitSwitch.setName("intakeLimitSwitch", hwMap);

		spinners.setName("intakeSpinners", hwMap);
		spinners.setDirection(DcMotorSimple.Direction.FORWARD);
		spinners.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		spinners.setPower(spinnersState.getPower());

		leftBeam.setName("intakeLeftBeam", hwMap);
		rightBeam.setName("intakeRightBeam", hwMap);
	}

	public void angleToCollect() {
		angleState = IntakeStates.Angle.COLLECT;
	}

	public void angleToTransfer() {
		angleState = IntakeStates.Angle.TRANSFER;
	}

	public void slidersRetract() {
		extensionState = IntakeStates.Extension.IN;
	}

	public void slidersExtend() {
		extensionState = IntakeStates.Extension.OUT;
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

	public void update() {
		if (leftBeam.isBroken() && rightBeam.isBroken() && angleState == IntakeStates.Angle.COLLECT) {
			angleState = IntakeStates.Angle.TRANSFER;
		}

		angle.setPosition(angleState.getPos());



		spinners.setPower(spinnersState.getPower());
	}

}
