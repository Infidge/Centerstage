package org.firstinspires.ftc.teamcode.hardware.subsystem;

import org.firstinspires.ftc.teamcode.util.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.constants.V4BConstants;

public class V4BStates {

    enum ArmAngle {
        DEPOSIT(V4BConstants.ARM_ANGLE_DEPOSIT),
        TRANSFER(V4BConstants.ARM_ANGLE_TRANSFER);

        private final double val;

        ArmAngle(double val) {
            this.val = val;
        }

        double getLeftPos() {
            return val;
        }

        double getRightPos() {
            return 1 - val;
        }
    }

    enum ClawAngle {
        DEPOSIT(V4BConstants.CLAW_ANGLE_DEPOSIT),
        TRANSFER(V4BConstants.CLAW_ANGLE_TRANSFER);

        private final double val;

        ClawAngle(double val) {
            this.val = val;
        }

        double getPos() {
            return val;
        }
    }

}
