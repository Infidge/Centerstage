package org.firstinspires.ftc.teamcode.Hardware.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.OptimisedHardware.OptimisedMotor;
import org.firstinspires.ftc.teamcode.Hardware.OptimisedHardware.OptimisedServo;
import org.firstinspires.ftc.teamcode.Utils.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Utils.IRBreakBeam;

public class Intake {

	private OptimisedMotor horizontalExtension;
	private OptimisedMotor spinners;
	private OptimisedServo angle;
	private IRBreakBeam leftBeam;
	private IRBreakBeam rightBeam;

	private IntakeAngleState angleState = IntakeAngleState.TRANSFER;

	private enum IntakeAngleState {
		COLLECT(IntakeConstants.ANGLE_COLLECT),
		TRANSFER(IntakeConstants.ANGLE_TRANSFER);

		private final double val;

		IntakeAngleState(double val) {
			this.val = val;
		}

		double get() {
			return val;
		}
	}

	public void init(HardwareMap hwMap) {
		horizontalExtension.setName("intakeHorizontalExtension", hwMap);
		horizontalExtension.setDirection(DcMotorSimple.Direction.FORWARD);
		horizontalExtension.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		horizontalExtension.setPower(0.0);

		spinners.setName("intakeSpinners", hwMap);
		spinners.setDirection(DcMotorSimple.Direction.FORWARD);
		spinners.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		spinners.setPower(0.0);

		angle.setName("intakeAngle", hwMap);
		angle.setPosition(angleState.get());

		leftBeam.setName("intakeLeftBeam", hwMap);
		rightBeam.setName("intakeRightBeam", hwMap);
	}

}
