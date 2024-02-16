package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotor;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;
import org.firstinspires.ftc.teamcode.hardware.common.BreakBeam;
import org.firstinspires.ftc.teamcode.motion.MotionProfile;
import org.firstinspires.ftc.teamcode.pidf.PDFController;
import org.firstinspires.ftc.teamcode.util.constants.IntakeConstants;

public class Intake {

	private final OptimisedServo angle = new OptimisedServo();
	private final OptimisedServo pixelCover = new OptimisedServo();
	private final OptimisedMotor spinners = new OptimisedMotor();

	private final OptimisedMotor horizontalExtension = new OptimisedMotor();
	private final LimitSwitch limitSwitch = new LimitSwitch();

	private final BreakBeam leftBeam = new BreakBeam();
	private final BreakBeam rightBeam = new BreakBeam();

	private IntakeStates.Angle angleState = IntakeStates.Angle.TRANSFER;
	private IntakeStates.PixelCover pixelCoverState = IntakeStates.PixelCover.LOWERED;
	private IntakeStates.Spinners spinnersState = IntakeStates.Spinners.CHILL;

	private IntakeStates.Extension extensionState = IntakeStates.Extension.IN;
	private IntakeStates.Extension lastExtensionState = extensionState;
	private final PDFController extensionPdfController = new PDFController(IntakeConstants.EXTENSION_P, IntakeConstants.EXTENSION_D, IntakeConstants.EXTENSION_F);
	private final MotionProfile extensionMotionProfile = new MotionProfile(IntakeConstants.EXTENSION_MAX_ACC, IntakeConstants.EXTENSION_MAX_DEC, IntakeConstants.EXTENSION_MAX_VEL);
	private boolean extensionMotionFinished = true;

	public Intake() {}

	public void init(HardwareMap hwMap) {
		angle.setName("intakeAngle", hwMap);
		angle.setPosition(angleState.getPos());

		pixelCover.setName("intakePixelCover", hwMap);
		pixelCover.setPosition(pixelCoverState.getPos());

		spinners.setName("intakeSpinners", hwMap);
		spinners.setDirection(DcMotorSimple.Direction.FORWARD);
		spinners.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		spinners.setPower(spinnersState.getPower());

		horizontalExtension.setName("intakeHorizontalExtension", hwMap);
		horizontalExtension.setDirection(DcMotorSimple.Direction.FORWARD);
		horizontalExtension.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		horizontalExtension.setPower(0.0);

		limitSwitch.setName("intakeLimitSwitch", hwMap);

		leftBeam.setName("intakeLeftBeam", hwMap);
		rightBeam.setName("intakeRightBeam", hwMap);
	}

	public void update() {
		if (leftBeam.isBroken() && rightBeam.isBroken() && angleState == IntakeStates.Angle.COLLECT) {
			angleState = IntakeStates.Angle.TRANSFER;
		}

		angle.setPosition(angleState.getPos());
		pixelCover.setPosition(pixelCoverState.getPos());
		spinners.setPower(spinnersState.getPower());

		if (extensionState != lastExtensionState) {
			extensionMotionProfile.start(extensionState.getPos());
			extensionMotionFinished = false;
			lastExtensionState = extensionState;
		}

		if (!extensionMotionFinished) {
			int extensionInstantPos = (int) extensionMotionProfile.update();

			if (extensionInstantPos == extensionMotionProfile.getTarget()) {
				extensionMotionFinished = true;
			} else {
				double extensionPower = extensionPdfController.update(extensionInstantPos, horizontalExtension.motor.getCurrentPosition());
				horizontalExtension.setPower(extensionPower);
			}
		}
	}

	public void angleToCollect() {
		angleState = IntakeStates.Angle.COLLECT;
	}

	public void angleToTransfer() {
		angleState = IntakeStates.Angle.TRANSFER;
	}

	public void pixelCoverLower() {
		pixelCoverState = IntakeStates.PixelCover.LOWERED;
	}

	public void pixelCoverRaise() {
		pixelCoverState = IntakeStates.PixelCover.RAISED;
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

	public void slidersRetract() {
		extensionState = IntakeStates.Extension.IN;
	}

	public void slidersExtend() {
		extensionState = IntakeStates.Extension.OUT;
	}

}
