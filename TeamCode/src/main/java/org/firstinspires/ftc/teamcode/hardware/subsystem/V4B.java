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

    private final PDFController pdfController = new PDFController(V4BConstants.ARM_P, V4BConstants.ARM_D, V4BConstants.ARM_F);
    private final MotionProfile motionProfile = new MotionProfile(V4BConstants.ARM_MAX_ACC, V4BConstants.ARM_MAX_DEC, V4BConstants.ARM_MAX_VEL);

    public static double DISTANCE = 0.75;
    public double instantPosition = 0;

    private V4BState state = V4BState.DEPOSIT;
    private V4BState lastState = state;
//    public boolean stateChanged = false;

    public V4B() {}

    public void init(HardwareMap hwMap) {
        armAngleLeft.setName("armAngleLeft", hwMap);
//        armAngleLeft.setPosition(state.getArmAngleLeftPos());

        armAngleRight.setName("armAngleRight", hwMap);
//        armAngleRight.setPosition(state.getArmAngleRightPos());

        clawAngle.setName("clawAngle", hwMap);
//        clawAngle.setPosition(state.getClawAnglePos());
    }

    public void update(Telemetry telemetry) {
//        armAngleLeft.setPosition(state.getArmAngleLeftPos());
//        armAngleRight.setPosition(state.getArmAngleRightPos());
//        clawAngle.setPosition(state.getClawAnglePos());

        /*pdfController.setCoefficients(V4BConstants.ARM_P, V4BConstants.ARM_D, V4BConstants.ARM_F);

        if (lastState != state) {
            motionProfile.setConstraints(V4BConstants.ARM_MAX_ACC, V4BConstants.ARM_MAX_DEC, V4BConstants.ARM_MAX_VEL);

            double dx = lastState.getArmAngleLeftPos() - state.getArmAngleLeftPos();
            boolean reverse = false;
            if (dx < 0) {
                dx = -dx;
                reverse = true;
            }

            motionProfile.setDistance(dx);
            motionProfile.start();
        }

        instantPosition = motionProfile.getInstantPosition(telemetry);

        if (instantPosition == state.getArmAngleLeftPos()) {
            lastState = state;
        }

        double armLeftPosition = instantPosition + state.getArmAngleLeftPos();

        armAngleLeft.setPosition(armLeftPosition);
        armAngleRight.setPosition(1.0 - armLeftPosition);*/
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
