package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Centerstage")
public class TeleOp extends LinearOpMode {

	private final Gamepad previousGamepad1 = new Gamepad();
	private final Gamepad previousGamepad2 = new Gamepad();

	private void copyGamepads() {
		previousGamepad1.copy(gamepad1);
		previousGamepad2.copy(gamepad2);
	}

	@Override
	public void runOpMode() throws InterruptedException {
		Robot robot = Robot.getInstance();
		robot.init(hardwareMap);

		Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

		ElapsedTime loopTime = new ElapsedTime();

		Gamepad.RumbleEffect errorRumbleEffect = new Gamepad.RumbleEffect.Builder()
				.addStep(1.0, 0.0, 125)
				.addStep(0.0, 1.0, 125)
				.addStep(1.0, 0.0, 125)
				.addStep(0.0, 1.0, 125)
				.build();

		copyGamepads();

		waitForStart();

		while (opModeIsActive()) {
			// Gamepad 1
			if (gamepad1.right_bumper) {
				robot.intake.spinInwards();
//				robot.intake.angleLower();
//				robot.intake.pixelCoverLower();
			} else if (gamepad1.left_bumper) {
				robot.intake.spinOutwards();
			} else {
				robot.intake.spinStop();
			}

			if (gamepad1.a) {
				robot.intake.slidersRetract();
			}

			if (gamepad1.b) {
				robot.intake.slidersExtend();
			}

//			if (gamepad1.y) {
//				robot.intake.angleRaise();
//				robot.intake.pixelCoverRaise();
//			}

			robot.drivetrain.update(gamepad1);

			// Gamepad 2
//			if (gamepad2.a && !previousGamepad2.a) {
//				robot.v4b.togglePosition();
//			}
//
//			if (gamepad2.b) {
//				robot.claw.pixelLeftOpen();
//				robot.claw.pixelRightOpen();
//			}
//
			if (gamepad2.left_bumper) {
				robot.lift.lower();
			}

			if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
				if (robot.lift.isRaised()) {
					robot.lift.pixelLevelIncrement();
				} else {
					robot.lift.raise();
				}
			}

			if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
				robot.lift.pixelLevelDecrement();
			}

			copyGamepads();

			telemetry.addData("intakeMotorPos", robot.intake.slides.motor.getCurrentPosition());
			telemetry.addData("intakeMotorPower", robot.intake.slides.motor.getPower());
			telemetry.addData("intakeLimitSwitch", robot.intake.limitSwitch.isPressed());
			telemetry.update();

			dashboardTelemetry.addData("intakeMotorPos", robot.intake.slides.motor.getCurrentPosition());
			dashboardTelemetry.addData("intakeMotorPower", robot.intake.slides.motor.getPower());
			dashboardTelemetry.addData("intakeLimitSwitch", robot.intake.limitSwitch.isPressed());
			dashboardTelemetry.addData("state", robot.intake.getSlidesState());
			dashboardTelemetry.update();

			robot.update();
		}
	}

}
