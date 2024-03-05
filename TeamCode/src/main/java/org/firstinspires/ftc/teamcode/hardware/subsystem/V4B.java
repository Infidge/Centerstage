package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;
import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.V4BConstants;
import org.firstinspires.ftc.teamcode.motion.MotionProfile;
import org.firstinspires.ftc.teamcode.pidf.PDFController;

public class V4B {

    public final OptimisedServo armAngleLeft = new OptimisedServo();
    public final OptimisedServo armAngleRight = new OptimisedServo();
    public final OptimisedServo clawAngle = new OptimisedServo();

    private final MotionProfile motionProfile = new MotionProfile(V4BConstants.ARM_MAX_ACC, V4BConstants.ARM_MAX_DEC, V4BConstants.ARM_MAX_VEL);

    private V4BState state = V4BState.DEPOSIT;
    private V4BState lastState = state;
    public boolean stateChanged = false;

    private boolean motionProfileReversed = false;
    private boolean motionProfileStopped = false;
    private double motionProfilePositionAtStop = 0.0;

    public V4B() {}

    public void init(HardwareMap hwMap) {
        armAngleLeft.setName("armAngleLeft", hwMap);
        armAngleLeft.setPosition(state.getArmAngleLeftPos());

        armAngleRight.setName("armAngleRight", hwMap);
        armAngleRight.setPosition(state.getArmAngleRightPos());

        clawAngle.setName("clawAngle", hwMap);
        clawAngle.setPosition(state.getClawAnglePos());
    }

    public void update(Telemetry telemetry) {
        clawAngle.setPosition(state.getClawAnglePos());

        if (stateChanged) {
            double motionProfileDistance = 0.0;
            if (motionProfile.isFinished()) {
                motionProfile.setConstraints(V4BConstants.ARM_MAX_ACC, V4BConstants.ARM_MAX_DEC, V4BConstants.ARM_MAX_VEL);

                motionProfileStopped = false;
                motionProfileDistance = state.getArmAngleLeftPos() - lastState.getArmAngleLeftPos();
            } else {
                motionProfileStopped = true;
                motionProfilePositionAtStop = armAngleLeft.getPosition();
                motionProfileDistance = state.getArmAngleLeftPos() - armAngleLeft.getPosition();
            }

            if (motionProfileDistance < 0) {
                motionProfileDistance = -motionProfileDistance;
                motionProfileReversed = true;
            } else {
                motionProfileReversed = false;
            }

            motionProfile.setDistance(motionProfileDistance);
            motionProfile.start(telemetry);

            stateChanged = false;
        }

        if (motionProfile.isFinished()) {
            armAngleLeft.setPosition(state.getArmAngleLeftPos());
            armAngleRight.setPosition(state.getArmAngleRightPos());
            lastState = state;
        } else {
            double startPosition;
            if (motionProfileStopped) {
                startPosition = motionProfilePositionAtStop;
            } else {
                startPosition = lastState.getArmAngleLeftPos();
            }

            double instantPosition = motionProfile.getInstantPosition();

            double armPosition;
            if (motionProfileReversed) {
                armPosition = startPosition - instantPosition;
            } else {
                armPosition = startPosition + instantPosition;
            }

            armAngleLeft.setPosition(armPosition);
            armAngleRight.setPosition(1 - armPosition);
        }

        telemetry.addData("instant", motionProfile.getInstantPosition());

    }

    public void toDepositPosition() {
        state = V4BState.DEPOSIT;
        stateChanged = true;
    }

    public void toTransferPosition() {
        state = V4BState.TRANSFER;
        stateChanged = true;
    }

    public void togglePosition() {
        if (state == V4BState.TRANSFER) {
            state = V4BState.DEPOSIT;
        } else if (state == V4BState.DEPOSIT) {
            state = V4BState.TRANSFER;
        }
        stateChanged = true;
    }

}
