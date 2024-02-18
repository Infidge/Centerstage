package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;

public class V4B {

    private final OptimisedServo armAngleLeft = new OptimisedServo();
    private final OptimisedServo armAngleRight = new OptimisedServo();
    private final OptimisedServo clawAngle = new OptimisedServo();

    private V4BState state = V4BState.DEPOSIT;

    public V4B() {}

    public void init(HardwareMap hwMap) {
        armAngleLeft.setName("armAngleLeft", hwMap);
        armAngleLeft.setPosition(state.getArmAngleLeftPos());

        armAngleRight.setName("armAngleRight", hwMap);
        armAngleRight.setPosition(state.getArmAngleRightPos());

        clawAngle.setName("clawAngle", hwMap);
        clawAngle.setPosition(state.getClawAnglePos());
    }

    public void update() {
        armAngleLeft.setPosition(state.getArmAngleLeftPos());
        armAngleRight.setPosition(state.getArmAngleRightPos());
        clawAngle.setPosition(state.getClawAnglePos());
    }

    public boolean isInTransferPosition() {
        return state == V4BState.TRANSFER;
    }

    public void toDepositPosition() {
        state = V4BState.DEPOSIT;
    }

    public void toTransferPosition() {
        state = V4BState.TRANSFER;
    }

    public void togglePosition() {
        if (state == V4BState.TRANSFER) {
            state = V4BState.DEPOSIT;
        } else if (state == V4BState.DEPOSIT) {
            state = V4BState.TRANSFER;
        }
    }

}
