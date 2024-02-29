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

import java.util.List;

public class Drivetrain {

	public final OptimisedMotor frontLeft = new OptimisedMotor();
	public final OptimisedMotor rearLeft = new OptimisedMotor();
	public final OptimisedMotor rearRight = new OptimisedMotor();
	public final OptimisedMotor frontRight = new OptimisedMotor();

	private final OptimisedMotor[] motors = new OptimisedMotor[] {frontLeft, rearLeft, frontRight, rearRight};
	public final double[] powers = new double[4];

	private LynxModule controlHub;
	private BNO055IMU imu;

	private static final double NOMINAL_VOLTAGE = 12.0;

	private final PDFController headingPdf = new PDFController(0.0, 0.0, 0.0);
	private double targetHeading;

	private final Gamepad previousGamepad = new Gamepad();

	public Drivetrain() {}

	public void init(HardwareMap hwMap) {
		frontLeft.setName("frontLeft", hwMap);
		rearLeft.setName("rearLeft", hwMap);
		frontRight.setName("frontRight", hwMap);
		rearRight.setName("rearRight", hwMap);

		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
		rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

		for (OptimisedMotor motor : motors) {
			motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			motor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
			motor.setPower(0.0);
		}

		List<LynxModule> lynxModules = hwMap.getAll(LynxModule.class);
		controlHub = lynxModules.stream().filter(LynxModule::isParent).findFirst().orElse(null);

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

		/* Heading PDF */
		boolean headingControl = false;
		if (turn == 0) {
			heading = imu.getAngularOrientation().firstAngle;
			headingControl = true;
		}

		if (stoppedSteering(gamepad))
			targetHeading = heading;

		if (headingControl)
			turn += headingPdf.update(targetHeading, heading);

		/* Mecanum kinematics */
		powers[0] = drive + strafe + turn;
		powers[1] = drive - strafe + turn;
		powers[2] = drive - strafe - turn;
		powers[3] = drive + strafe - turn;

		powerRegulation();

		for (int i = 0; i < 4; i++) {
			motors[i].setPower(powers[i]);
		}

		previousGamepad.copy(gamepad);
	}

	private boolean stoppedSteering(Gamepad gamepad) {
		return (previousGamepad.left_trigger - previousGamepad.right_trigger) != 0.0 &&
				(gamepad.left_trigger - gamepad.right_trigger) == 0;
	}

	private void powerRegulation() {
		double maxPower = Math.max(Math.abs(powers[0]), Math.max(Math.abs(powers[1]), Math.max(Math.abs(powers[2]), Math.abs(powers[3]))));

		if (maxPower > 1.0) {
			for (int i = 0; i < 4; i++) {
				powers[i] = powers[i] / maxPower;
				powers[i] = powers[i] * NOMINAL_VOLTAGE / controlHub.getInputVoltage(VoltageUnit.VOLTS);
			}
		}
	}

}
