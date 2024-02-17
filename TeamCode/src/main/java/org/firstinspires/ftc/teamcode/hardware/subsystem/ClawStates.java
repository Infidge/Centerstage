package org.firstinspires.ftc.teamcode.hardware.subsystem;

import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.ClawConstants;

public class ClawStates {

    enum PixelLeft {
        CLOSED(ClawConstants.PIXEL_LEFT_CLOSED),
        OPEN(ClawConstants.PIXEL_LEFT_OPEN);

        private final double val;

        PixelLeft(double val) {
            this.val = val;
        }

        double getPos() {
            return val;
        }
    }

    enum PixelRight {
        CLOSED(ClawConstants.PIXEL_RIGHT_CLOSED),
        OPEN(ClawConstants.PIXEL_RIGHT_OPEN);

        private final double val;

        PixelRight(double val) {
            this.val = val;
        }

        double getPos() {
            return val;
        }
    }

    enum Rotation {
        HORIZONTAL(ClawConstants.ROTATION_HORIZONTAL),
        VERTICAL(ClawConstants.ROTATION_VERTICAL);

        private final double val;

        Rotation(double val) {
            this.val = val;
        }

        double getPos() {
            return val;
        }
    }

}
