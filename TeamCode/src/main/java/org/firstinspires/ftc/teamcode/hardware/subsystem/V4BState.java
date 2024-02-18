package org.firstinspires.ftc.teamcode.hardware.subsystem;

import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.V4BConstants;

enum V4BState {

    DEPOSIT(V4BConstants.ARM_ANGLE_DEPOSIT, V4BConstants.CLAW_ANGLE_DEPOSIT),
    TRANSFER(V4BConstants.ARM_ANGLE_TRANSFER, V4BConstants.CLAW_ANGLE_TRANSFER);

    private final double armAnglePos, clawAnglePos;

    V4BState(double armAnglePos, double clawAnglePos) {
        this.armAnglePos = armAnglePos;
        this.clawAnglePos = clawAnglePos;
    }

    public double getArmAngleLeftPos() {
        return armAnglePos;
    }

    public double getArmAngleRightPos() {
        return 1 - armAnglePos;
    }

    public double getClawAnglePos() {
        return clawAnglePos;
    }

}
