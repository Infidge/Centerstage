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
import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.IntakeConstants;

public class Intake {

	private final OptimisedServo angle = new OptimisedServo();
	private final OptimisedServo pixelCover = new OptimisedServo();
	public final OptimisedMotor slides = new OptimisedMotor();
	private final OptimisedMotor spinners = new OptimisedMotor();

	public final LimitSwitch limitSwitch = new LimitSwitch();

	private final BreakBeam leftBeam = new BreakBeam();
	private final BreakBeam rightBeam = new BreakBeam();

	private IntakeStates.Angle angleState = IntakeStates.Angle.RAISED;
	private IntakeStates.PixelCover pixelCoverState = IntakeStates.PixelCover.LOWERED;
	private IntakeStates.Spinners spinnersState = IntakeStates.Spinners.STOP;

	private IntakeStates.Sliders slidesStates = IntakeStates.Sliders.RETRACT;
	private IntakeStates.Sliders lastSlidesStates = slidesStates;
	private final PDFController slidesPdfController = new PDFController(IntakeConstants.SLIDES_P, IntakeConstants.SLIDES_D, IntakeConstants.SLIDES_F);
	private final MotionProfile slidesMotionProfile = new MotionProfile(IntakeConstants.SLIDES_MAX_ACC, IntakeConstants.SLIDES_MAX_DEC, IntakeConstants.SLIDES_MAX_VEL);

	public Intake() {}

	public void init(HardwareMap hwMap) {
		angle.setName("intakeAngle", hwMap);
		angle.setPosition(angleState.getPos());

		pixelCover.setName("intakePixelCover", hwMap);
		pixelCover.setPosition(pixelCoverState.getPos());

		slides.setName("intakeSlides", hwMap);
		slides.setDirection(DcMotorSimple.Direction.REVERSE);
		slides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		slides.setPower(0.0);

		spinners.setName("intakeSpinners", hwMap);
		spinners.setDirection(DcMotorSimple.Direction.FORWARD);
		spinners.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		spinners.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		spinners.setPower(spinnersState.getPower());

		limitSwitch.setName("intakeLimitSwitch", hwMap);

//		leftBeam.setName("intakeLeftBeam", hwMap);
//		rightBeam.setName("intakeRightBeam", hwMap);
	}

	public void update() {
//		if (leftBeam.isBroken() && rightBeam.isBroken()) {
//			angleRaise();
//			pixelCoverRaise();
//		}

		angle.setPosition(angleState.getPos());
		pixelCover.setPosition(pixelCoverState.getPos());
		spinners.setPower(spinnersState.getPower());

		if (slidesStates == IntakeStates.Sliders.RETRACT) {
			if (limitSwitch.isPressed()) {
//				slides.setPower(0.0);
//				slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//				slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			} else {
//				slides.setPower(-1.0);
			}
		} else if (slidesStates == IntakeStates.Sliders.EXTEND) {
//			boolean reverseSlidersPower = false;
//
//			if (slidersState != lastSlidersState) {
//				double slidersTargetPosition = slidersState.getPos();
//				double slidersCurrentPosition = sliders.motor.getCurrentPosition();
//				double slidersDistance = slidersTargetPosition - slidersCurrentPosition;
//
//				if (slidersDistance < 0) {
//					reverseSlidersPower = true;
//					slidersDistance = -slidersDistance;
//				}
//
//				slidersMotionProfile.setDistance(slidersDistance);
//				slidersMotionProfile.start();
//				lastSlidersState = slidersState;
//			}
//
//			int slidersInstantTargetPosition = (int) slidersMotionProfile.getInstantPosition();
//
//			double slidersPower = slidersPdfController.update(slidersInstantTargetPosition, sliders.motor.getCurrentPosition());
//			if (reverseSlidersPower) {
//				slidersPower = -slidersPower;
//			}
//
//			sliders.setPower(slidersPower);
		}
	}

	public void angleLower() {
		angleState = IntakeStates.Angle.LOWERED;
	}

	public void angleRaise() {
		angleState = IntakeStates.Angle.RAISED;
	}

	public void pixelCoverLower() {
		pixelCoverState = IntakeStates.PixelCover.LOWERED;
	}

	public void pixelCoverRaise() {
		pixelCoverState = IntakeStates.PixelCover.RAISED;
	}

	public void slidersRetract() {
		slidesStates = IntakeStates.Sliders.RETRACT;
	}

	public void slidersExtend() {
		slidesStates = IntakeStates.Sliders.EXTEND;
	}

	public void spinInwards() {
		spinnersState = IntakeStates.Spinners.IN;
	}

	public void spinOutwards() {
		spinnersState = IntakeStates.Spinners.OUT;
	}

	public void spinStop() {
		spinnersState = IntakeStates.Spinners.STOP;
	}

}
