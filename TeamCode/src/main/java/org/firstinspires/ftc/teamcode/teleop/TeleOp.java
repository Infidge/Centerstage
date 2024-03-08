package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		copyGamepads();

		waitForStart();

		while (opModeIsActive()) {
			// Gamepad 1
			if (gamepad1.right_bumper) {
				robot.intake.spinInwards();
				robot.intake.angleLower();
				robot.intake.pixelCoverLower();
			} else if (gamepad1.left_bumper) {
				robot.intake.spinOutwards();
			} else {
				robot.intake.spinStop();
			}

//			if (gamepad1.a) {
//				robot.intake.slidersRetract();
//			}

//			if (gamepad1.b) {
//				robot.intake.slidersExtend();
//			}

			if (gamepad1.y) {
				robot.intake.angleRaise();
				robot.intake.pixelCoverRaise();
			}

			robot.drivetrain.update(gamepad1);

			// Gamepad 2
			if (gamepad2.a && !previousGamepad2.a) {
				// if pixel cover isnt lowered, dont move to wait on cover
				robot.v4b.togglePosition();
				//robot.v4b.toTransferPosition();
			}

			if (gamepad2.b && !previousGamepad2.b) {
				robot.requestTransfer();
				//robot.v4b.toWaitForCoverRaisePosition();
			}

			if (gamepad2.x && !previousGamepad2.x) {
				robot.claw.pixelLeftOpen();
				robot.claw.pixelRightOpen();
			}

//			if (robot.intake.isReadyForTransfer() && robot.v4b.isInTransferPosition()) {
//				robot.claw.pixelLeftClose();
//				robot.claw.pixelRightClose();
//
////				robot.v4b.toDepositPosition();
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

			robot.update(telemetry);
			telemetry.addData("armAngleLeftPosition", robot.v4b.armAngleLeft.getPosition());
			telemetry.addData("clawAnglePosition", robot.v4b.clawAngle.getPosition());
			telemetry.addData("hasPixels", robot.intake.hasPixels());
			telemetry.update();
		}
	}

}
