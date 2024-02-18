package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.common.LimitSwitch;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotor;
import org.firstinspires.ftc.teamcode.hardware.subsystem.constants.LiftConstants;
import org.firstinspires.ftc.teamcode.pidf.PDFController;

public class Lift {

    private final OptimisedMotor motor1 = new OptimisedMotor();
    private final OptimisedMotor motor2 = new OptimisedMotor();

    private final LimitSwitch limitSwitch = new LimitSwitch();

    private enum LiftState {
        STOP,
        LOWER,
        RAISE
    }

    private LiftState state = LiftState.STOP;

    private final PDFController pdfController = new PDFController(LiftConstants.LIFT_P, LiftConstants.LIFT_D, LiftConstants.LIFT_F);

    private int pixelLevel = LiftConstants.PIXEL_LEVEL_MIN;

    public void init(HardwareMap hwMap) {
        motor1.setName("liftMotorEncoder", hwMap);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setPower(0.0);

        motor2.setName("liftMotor", hwMap);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setPower(0.0);

        limitSwitch.setName("liftLimitSwitch", hwMap);
    }

    public void update() {
        if (state == LiftState.LOWER) {
            if (limitSwitch.isPressed()) {
                state = LiftState.STOP;
                motor1.setPower(0.0);
                motor2.setPower(0.0);
                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                motor1.setPower(LiftConstants.LOWER_POWER);
                motor2.setPower(LiftConstants.LOWER_POWER);
            }
        } else if (state == LiftState.RAISE) {
            pixelLevel = Range.clip(pixelLevel, LiftConstants.PIXEL_LEVEL_MIN, LiftConstants.PIXEL_LEVEL_MAX);

            int targetPosition = LiftConstants.PIXEL_LEVEL_BASE + LiftConstants.PIXEL_LEVEL_INCREMENT * pixelLevel;
            int power = pdfController.update(targetPosition, motor1.motor.getCurrentPosition());
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }

    public boolean isRaised() {
        return state == LiftState.RAISE;
    }

    public void lower() {
        state = LiftState.LOWER;
    }

    public void raise() {
        state = LiftState.RAISE;
    }

    public void pixelLevelDecrement() {
        pixelLevel--;
    }

    public void pixelLevelIncrement() {
        pixelLevel++;
    }

}
