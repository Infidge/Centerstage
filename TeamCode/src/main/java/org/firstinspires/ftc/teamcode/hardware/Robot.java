package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystem.V4B;

public class Robot {

	private static Robot instance = null;

	public final Drivetrain drivetrain;
	public final Intake intake;
	public final Claw claw;
	public final Lift lift;
	public final V4B v4b;

	private enum TransferStates {
		WAIT_INTAKE_TO_HAVE_PIXELS(0),
		WAIT_V4B_MOVE_TO_WAIT_FOR_COVER_RAISE(1),
		WAIT_PIXEL_COVER_TO_RAISE(2),
		WAIT_V4B_MOVE_TO_TRANSFER(3),
		WAIT_FOR_PIXEL_CLAWS_TO_CLOSE(4),
		WAIT_V4B_TO_DEPOSIT_INTER(5);
//		WAIT_V4B_MOVE_TO_DEPOSIT(6);

		public int id;

		TransferStates(int id) {
			this.id = id;
		}
	}

	private TransferStates state = TransferStates.WAIT_INTAKE_TO_HAVE_PIXELS;
	private final ElapsedTime time = new ElapsedTime();
	private boolean requestedTransfer = false;

	private Robot()	{
		drivetrain = new Drivetrain();
		intake = new Intake();
		claw = new Claw();
		lift = new Lift();
		v4b = new V4B();
	}

	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}

	public void init(HardwareMap hwMap) {
		drivetrain.init(hwMap);
		intake.init(hwMap);
		claw.init(hwMap);
		lift.init(hwMap);
		v4b.init(hwMap);
	}

	public void update(Telemetry telemetry) {
		intake.update();
		claw.update();
		lift.update();
		v4b.update(telemetry);

		telemetry.addData("requested", requestedTransfer);
		telemetry.addData("time", time.seconds());
		telemetry.addData("stateChanged", v4b.stateChanged);
		telemetry.addData("state: ", state);

		if (requestedTransfer) {
			switch (state) {
				case WAIT_INTAKE_TO_HAVE_PIXELS:
					if (intake.hasPixels() && v4b.isInWaitOnCoverPosition()) {
						state = TransferStates.WAIT_V4B_MOVE_TO_WAIT_FOR_COVER_RAISE;
						v4b.toWaitForCoverRaisePosition();
						claw.pixelLeftOpen();
						claw.pixelRightOpen();
						time.reset();
					} else {
						requestedTransfer = false;
					}
					break;
				case WAIT_V4B_MOVE_TO_WAIT_FOR_COVER_RAISE:
					if (time.seconds() > 0.2) {
						state = TransferStates.WAIT_PIXEL_COVER_TO_RAISE;
						intake.pixelCoverRaise();
						time.reset();
					}
					break;
				case WAIT_PIXEL_COVER_TO_RAISE:
					if (time.seconds() > 0.3) {
						state = TransferStates.WAIT_V4B_MOVE_TO_TRANSFER;
						v4b.toTransferPosition();
						time.reset();
					}
					break;
				case WAIT_V4B_MOVE_TO_TRANSFER:
					if (time.seconds() > 0.25) {
						state = TransferStates.WAIT_FOR_PIXEL_CLAWS_TO_CLOSE;
						claw.pixelLeftClose();
						claw.pixelRightClose();
						time.reset();
					}
					break;
				case WAIT_FOR_PIXEL_CLAWS_TO_CLOSE:
					if (time.seconds() > 0.15) {
						state = TransferStates.WAIT_INTAKE_TO_HAVE_PIXELS;
						v4b.toWaitBeforeDepositPosition();
						time.reset();
						requestedTransfer = false;
					}
					break;
//				case WAIT_V4B_TO_DEPOSIT_INTER:
//					if (time.seconds() > 0.2) {
//						state = TransferStates.WAIT_V4B_MOVE_TO_DEPOSIT;
//						v4b.toDepositPosition();
//						time.reset();
//					}
//				case WAIT_V4B_MOVE_TO_DEPOSIT:
//					if (time.seconds() > 0.3) {
//						state = TransferStates.WAIT_INTAKE_TO_HAVE_PIXELS;
//						intake.pixelCoverLower();
//						requestedTransfer = false;
//					}
//					break;
			}
		}
	}

	public void requestTransfer() {
		requestedTransfer = true;
	}
}
