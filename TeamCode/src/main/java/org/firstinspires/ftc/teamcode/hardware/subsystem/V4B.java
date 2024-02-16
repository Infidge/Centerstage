package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;

public class V4B {

    private final OptimisedServo armAngleLeft = new OptimisedServo();
    private final OptimisedServo armAngleRight = new OptimisedServo();
    private final OptimisedServo clawAngle = new OptimisedServo();

    private V4BStates.ArmAngle armAngleState = V4BStates.ArmAngle.TRANSFER;
    private V4BStates.ClawAngle clawAngleState = V4BStates.ClawAngle.TRANSFER;

    public V4B() {}

    public void init(HardwareMap hwMap) {
        armAngleLeft.setName("armAngleLeft", hwMap);
        armAngleLeft.setPosition(armAngleState.getLeftPos());

        armAngleRight.setName("armAngleRight", hwMap);
        armAngleRight.setPosition(armAngleState.getRightPos());

        clawAngle.setName("clawAngle", hwMap);
        clawAngle.setPosition(clawAngleState.getPos());
    }

    public void update() {
        armAngleLeft.setPosition(armAngleState.getLeftPos());
        armAngleRight.setPosition(armAngleState.getRightPos());
        clawAngle.setPosition(clawAngleState.getPos());
    }

    public void toTransferPosition() {
        armAngleState = V4BStates.ArmAngle.TRANSFER;
        clawAngleState = V4BStates.ClawAngle.TRANSFER;
    }

    public void toDepositPosition() {
        armAngleState = V4BStates.ArmAngle.DEPOSIT;
        clawAngleState = V4BStates.ClawAngle.DEPOSIT;
    }

}
