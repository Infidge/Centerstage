package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;
import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.V4BConstants;
import org.firstinspires.ftc.teamcode.motion.MotionProfile;

public class V4B {

    public final OptimisedServo armAngleLeft = new OptimisedServo();
    public final OptimisedServo armAngleRight = new OptimisedServo();
    public final OptimisedServo clawAngle = new OptimisedServo();

    private V4BState state = V4BState.DEPOSIT;
    private V4BState lastState = state;
    public boolean stateChanged = false;

    private final MotionProfile armMotionProfile = new MotionProfile(V4BConstants.ARM_MAX_ACC, V4BConstants.ARM_MAX_DEC, V4BConstants.ARM_MAX_VEL);
    private boolean armMotionProfileReversed = false;

    private final MotionProfile clawMotionProfile = new MotionProfile(V4BConstants.CLAW_MAX_ACC, V4BConstants.CLAW_MAX_DEC, V4BConstants.CLAW_MAX_VEL);
    private boolean clawMotionProfileReversed = false;

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
        if (stateChanged && armMotionProfile.isFinished() && clawMotionProfile.isFinished()) {
            armMotionProfile.setConstraints(V4BConstants.ARM_MAX_ACC, V4BConstants.ARM_MAX_DEC, V4BConstants.ARM_MAX_VEL);

            double armMotionProfileDistance = state.getArmAngleLeftPos() - lastState.getArmAngleLeftPos();
            if (armMotionProfileDistance < 0) {
                armMotionProfileDistance = -armMotionProfileDistance;
                armMotionProfileReversed = true;
            } else {
                armMotionProfileReversed = false;
            }

            armMotionProfile.setDistance(armMotionProfileDistance);
            armMotionProfile.start(telemetry);

            clawMotionProfile.setConstraints(V4BConstants.CLAW_MAX_ACC, V4BConstants.CLAW_MAX_DEC, V4BConstants.CLAW_MAX_VEL);

            double clawMotionProfileDistance = state.getClawAnglePos() - lastState.getClawAnglePos();

            if (clawMotionProfileDistance < 0) {
                clawMotionProfileDistance = -clawMotionProfileDistance;
                clawMotionProfileReversed = true;
            } else {
                clawMotionProfileReversed = false;
            }

            clawMotionProfile.setDistance(clawMotionProfileDistance);
            clawMotionProfile.start(telemetry);

            stateChanged = false;
        }

        if (armMotionProfile.isFinished() && clawMotionProfile.isFinished()) {
            armAngleLeft.setPosition(state.getArmAngleLeftPos());
            armAngleRight.setPosition(state.getArmAngleRightPos());
            clawAngle.setPosition(state.getClawAnglePos());

            lastState = state;
        } else {
            double armStartPosition = lastState.getArmAngleLeftPos();
            double armInstantPosition = armMotionProfile.getInstantPosition();

            double armPosition;
            if (armMotionProfileReversed) {
                armPosition = armStartPosition - armInstantPosition;
            } else {
                armPosition = armStartPosition + armInstantPosition;
            }

            armAngleLeft.setPosition(armPosition);
            armAngleRight.setPosition(1 - armPosition);

            double clawStartPosition = lastState.getClawAnglePos();
            double clawInstantPosition = clawMotionProfile.getInstantPosition();

            double clawPosition;
            if (clawMotionProfileReversed) {
                clawPosition = clawStartPosition - clawInstantPosition;
            } else {
                clawPosition = clawStartPosition + clawInstantPosition;
            }

            clawAngle.setPosition(clawPosition);
        }

    }

    public void toDepositPosition() {
        state = V4BState.DEPOSIT;
        stateChanged = true;
    }

    public void toWaitOnCoverPosition() {
        state = V4BState.WAIT_ON_COVER;
        stateChanged = true;
    }

    public void toWaitForCoverRaisePosition() {
        state = V4BState.WAIT_FOR_COVER_RAISE;
        stateChanged = true;
    }

    public void toTransferPosition() {
        state = V4BState.TRANSFER;
        stateChanged = true;
    }

    public void togglePosition() {
        if (state == V4BState.WAIT_ON_COVER) {
            state = V4BState.DEPOSIT;
        } else if (state == V4BState.DEPOSIT) {
            state = V4BState.WAIT_ON_COVER;
        }
        stateChanged = true;
    }

    public boolean isInWaitOnCoverPosition() {
        return state == V4BState.WAIT_ON_COVER;
    }
}
