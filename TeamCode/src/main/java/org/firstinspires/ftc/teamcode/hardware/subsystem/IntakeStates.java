package org.firstinspires.ftc.teamcode.hardware.subsystem;

import org.firstinspires.ftc.teamcode.util.constants.IntakeConstants;

class IntakeStates {

	enum Angle {
		COLLECT(IntakeConstants.ANGLE_COLLECT),
		TRANSFER(IntakeConstants.ANGLE_TRANSFER);

		private final double val;

		Angle(double val) {
			this.val = val;
		}

		double getPos() {
			return val;
		}
	}

	enum Extension {
		IN(0),
		OUT(IntakeConstants.HORIZONTAL_EXTENSION_OUT);

		private final int val;

		Extension(int val) {
			this.val = val;
		}

		double getPos() {
			return val;
		}
	}

	enum Spinners {
		CHILL(0.0),
		IN(IntakeConstants.SPINNERS_IN_POWER),
		OUT(IntakeConstants.SPINNERS_OUT_POWER),
		TRANSFER(IntakeConstants.SPINNERS_TRANSFER_POWER);

		private final double val;

		Spinners(double val) {
			this.val = val;
		}

		double getPower() {
			return val;
		}
	}

}
