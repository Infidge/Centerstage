package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class motorTest extends LinearOpMode {

	DcMotorEx motor;
	@Override
	public void runOpMode()
	{
		motor = hardwareMap.get(DcMotorEx.class, "motor");
		motor.setDirection(DcMotorSimple.Direction.FORWARD);
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setPower(0.0);

		waitForStart();

		while (opModeIsActive()) {
			if (gamepad1.right_trigger > 0.0)
				motor.setPower(1.0);
			else if (gamepad1.left_trigger > 0.0)
				motor.setPower(-1.0);
			else motor.setPower(0.0);
		}
	}
}
