package org.firstinspires.ftc.teamcode.hardware.common;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class OptimisedMotorByBaciu extends DcMotorImplEx {

    private double lastPower = 2.0;
    private static final double EPSILON = 0.01;

    public OptimisedMotorByBaciu(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public OptimisedMotorByBaciu(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public OptimisedMotorByBaciu(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }

    @Override
    public synchronized void setPower(double power) {
        if (Math.abs(lastPower - power) > EPSILON) {
            super.setPower(power);
            lastPower = power;
        }
    }

}
