package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotor;
import org.firstinspires.ftc.teamcode.pidf.PDFController;
import org.firstinspires.ftc.teamcode.util.LoopTime;

public class Drivetrain {

	private final OptimisedMotor frontLeft = new OptimisedMotor();
	private final OptimisedMotor rearLeft = new OptimisedMotor();
	private final OptimisedMotor rearRight = new OptimisedMotor();
	private final OptimisedMotor frontRight = new OptimisedMotor();

	private BNO055IMU imu;

	private LynxModule voltageRegulator;

	private Gamepad currentGamepad;
	private Gamepad prevGamepad;

	private final PDFController headingPDF = new PDFController(0.0, 0.0, 0.0);

	private final LoopTime loopTime = new LoopTime();

	private double targetHeading;
	private boolean headingControl = false;

	public Drivetrain() {}

	public void init(HardwareMap hwMap) {
		frontLeft.setName("front_left", hwMap);
		rearLeft.setName("rear_left", hwMap);
		rearRight.setName("rear_right", hwMap);
		frontRight.setName("front_right", hwMap);

		frontLeft.setPower(0.0);
		rearLeft.setPower(0.0);
		rearRight.setPower(0.0);
		frontRight.setPower(0.0);

		frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
		frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

		imu = hwMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
	}

	public void update(Gamepad gamepad) {
		double drive = -gamepad.left_stick_y;
		double strafe = gamepad.left_stick_x;
		double turn = gamepad.right_trigger - gamepad.left_trigger;
		double heading = 0.0;

		/** Heading PDF */
		if (turn == 0) {
			heading = imu.getAngularOrientation().firstAngle;
			headingControl = true;
		} else headingControl = false;

		if (stoppedSteering())
			targetHeading = heading;

		if (headingControl)
			turn += headingPDF.update(targetHeading, heading);

		/** Mecanum kinematics */
		double flPower = drive + strafe + turn;
		double rlPower = drive - strafe + turn;
		double rrPower = drive + strafe - turn;
		double frPower = drive - strafe - turn;

		powerRegulation(flPower, rlPower, rrPower, frPower);

		frontLeft.setPower(flPower);
		rearLeft.setPower(rlPower);
		rearRight.setPower(rrPower);
		frontRight.setPower(frPower);
	}

	boolean stoppedSteering() {
		return (prevGamepad.left_trigger - prevGamepad.right_trigger) != 0.0 &&
				(currentGamepad.left_trigger - currentGamepad.right_trigger) == 0;
	}

	private void powerRegulation (double power1, double power2, double power3, double power4) {
		double max = Math.max(Math.abs(power1), Math.max(Math.abs(power2), Math.max(Math.abs(power3), Math.abs(power4))));

		/** Power wrapping */
		if (max > 1.0) {
			power1 /= max;
			power2 /= max;
			power3 /= max;
			power4 /= max;
		}

		/** Voltage correction */
		power1 *= 12.0 / voltageRegulator.getInputVoltage(VoltageUnit.VOLTS);
		power2 *= 12.0 / voltageRegulator.getInputVoltage(VoltageUnit.VOLTS);
		power3 *= 12.0 / voltageRegulator.getInputVoltage(VoltageUnit.VOLTS);
		power4 *= 12.0 / voltageRegulator.getInputVoltage(VoltageUnit.VOLTS);
	}

}
