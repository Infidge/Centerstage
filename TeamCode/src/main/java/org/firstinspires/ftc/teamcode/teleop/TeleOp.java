package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Robot;

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

		Gamepad.RumbleEffect errorRumbleEffect = new Gamepad.RumbleEffect.Builder()
				.addStep(1.0, 0.0, 125)
				.addStep(0.0, 1.0, 125)
				.addStep(1.0, 0.0, 125)
				.addStep(0.0, 1.0, 125)
				.build();

		copyGamepads();

		waitForStart();

		while (opModeIsActive()) {
			copyGamepads();

			// Automatic movements (should be placed before any gamepad controls, so that gamepads have precedence)
			if (robot.intake.arePixelsIn()) {
				robot.v4b.toTransferPosition();
			}

			if (robot.claw.arePixelsCaught()) {
				robot.v4b.toDepositPosition();
			}

			// Gamepad 1
			if (gamepad1.right_bumper) {
				robot.intake.spinInwards();
				robot.intake.angleLower();

				if (robot.v4b.isInTransferPosition()) {
					gamepad1.runRumbleEffect(errorRumbleEffect);
				} else {
					robot.intake.pixelCoverLower();
				}

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

			robot.drivetrain.update(gamepad1);

			// Gamepad 2
			if (gamepad2.a) {
				robot.v4b.toDepositPosition();
			}

			if (gamepad2.b) {
				robot.v4b.toTransferPosition();
			}

			if (gamepad2.x) {
				robot.claw.pixelLeftClose();
				robot.claw.pixelRightClose();
			}

			if (gamepad2.y) {
				robot.claw.pixelLeftOpen();
				robot.claw.pixelRightOpen();
			}

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

			robot.update();
		}
	}

}
