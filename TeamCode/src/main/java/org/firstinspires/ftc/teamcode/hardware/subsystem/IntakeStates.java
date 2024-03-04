package org.firstinspires.ftc.teamcode.hardware.subsystem;

import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.IntakeConstants;

class IntakeStates {

	enum Angle {
		LOWERED(IntakeConstants.ANGLE_LOWERED),
		RAISED(IntakeConstants.ANGLE_RAISED);

		private final double val;

		Angle(double val) {
			this.val = val;
		}

		double getPos() {
			return val;
		}
	}

	enum PixelCover {
		LOWERED(IntakeConstants.PIXEL_COVER_LOWERED),
		RAISED(IntakeConstants.PIXEL_COVER_RAISED);

		private final double val;

		PixelCover(double val) {
			this.val = val;
		}

		double getPos() {
			return val;
		}
	}

	enum Sliders {
		RETRACT(0),
		RETRACT_HOLD(1),
		EXTEND(IntakeConstants.SLIDES_EXTENDED_POSITION);

		private final int val;

		Sliders(int val) {
			this.val = val;
		}

		double getPos() {
			return val;
		}
	}

	enum Spinners {
		STOP(0.0),
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
