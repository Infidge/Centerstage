package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotor;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotorByBaciu;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;
import org.firstinspires.ftc.teamcode.hardware.common.BreakBeam;
import org.firstinspires.ftc.teamcode.motion.MotionProfile;
import org.firstinspires.ftc.teamcode.pidf.PDFController;
import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.IntakeConstants;

public class Intake {

	private final OptimisedServo angle = new OptimisedServo();
	private final OptimisedServo pixelCover = new OptimisedServo();
	private final OptimisedMotor sliders = new OptimisedMotor();
	private OptimisedMotorByBaciu spinners;

	private final LimitSwitch limitSwitch = new LimitSwitch();

	private final BreakBeam leftBeam = new BreakBeam();
	private final BreakBeam rightBeam = new BreakBeam();

	private IntakeStates.Angle angleState = IntakeStates.Angle.LOWERED;
	private IntakeStates.PixelCover pixelCoverState = IntakeStates.PixelCover.LOWERED;
	private IntakeStates.Spinners spinnersState = IntakeStates.Spinners.STOP;

	private IntakeStates.Sliders slidersState = IntakeStates.Sliders.RETRACTED;
	private IntakeStates.Sliders lastSlidersState = slidersState;
	private final PDFController slidersPdfController = new PDFController(IntakeConstants.SLIDERS_P, IntakeConstants.SLIDERS_D, IntakeConstants.SLIDERS_F);
	private final MotionProfile slidersMotionProfile = new MotionProfile(IntakeConstants.SLIDERS_MAX_ACC, IntakeConstants.SLIDERS_MAX_DEC, IntakeConstants.SLIDERS_MAX_VEL);

	public Intake() {}

	public void init(HardwareMap hwMap) {
		angle.setName("intakeAngle", hwMap);
		angle.setPosition(angleState.getPos());

		pixelCover.setName("intakePixelCover", hwMap);
		pixelCover.setPosition(pixelCoverState.getPos());

		sliders.setName("intakeSliders", hwMap);
		sliders.setDirection(DcMotorSimple.Direction.FORWARD);
		sliders.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
		sliders.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		sliders.setPower(0.0);

		spinners = hwMap.get(OptimisedMotorByBaciu.class, "intakeSpinners");
		spinners.setDirection(DcMotorSimple.Direction.FORWARD);
		spinners.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		spinners.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		spinners.setPower(spinnersState.getPower());

		limitSwitch.setName("intakeLimitSwitch", hwMap);

		leftBeam.setName("intakeLeftBeam", hwMap);
		rightBeam.setName("intakeRightBeam", hwMap);
	}

	public void update() {
		if (leftBeam.isBroken() && rightBeam.isBroken()) {
			angleRaise();
			pixelCoverRaise();
		}

		angle.setPosition(angleState.getPos());
		pixelCover.setPosition(pixelCoverState.getPos());
		spinners.setPower(spinnersState.getPower());

		boolean reverseSlidersPower = false;

		if (slidersState != lastSlidersState) {
			double slidersTargetPosition = slidersState.getPos();
			double slidersCurrentPosition = sliders.motor.getCurrentPosition();
			double slidersDistance = slidersTargetPosition - slidersCurrentPosition;

			if (slidersDistance < 0) {
				reverseSlidersPower = true;
				slidersDistance = -slidersDistance;
			}

			slidersMotionProfile.setDistance(slidersDistance);
			slidersMotionProfile.start();
			lastSlidersState = slidersState;
		}

		int slidersInstantTargetPosition = (int) slidersMotionProfile.getInstantPosition();

		double slidersPower = slidersPdfController.update(slidersInstantTargetPosition, sliders.motor.getCurrentPosition());
		if (reverseSlidersPower) {
			slidersPower = -slidersPower;
		}
		sliders.setPower(slidersPower);
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
		slidersState = IntakeStates.Sliders.RETRACTED;
	}

	public void slidersExtend() {
		slidersState = IntakeStates.Sliders.EXTENDED;
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
