package org.firstinspires.ftc.teamcode.Hardware.OptimisedHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class OptimisedMotor {

    private DcMotorEx motor;

    private double lastPower = 2.0;
    private static final double EPSILON = 0.01;

    public OptimisedMotor() {}

    public void setName(String name, HardwareMap hwMap) {
        this.motor = hwMap.get(DcMotorEx.class, name);
    }

    public void setPower(Double power) {
        if (Math.abs(lastPower - power) > EPSILON) {
            this.motor.setPower(power);
            this.lastPower = power;
        }
    }

    public double getPower() {
        return this.lastPower;
    }

    public double getCurrent(CurrentUnit unit) {
        return this.motor.getCurrent(unit);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        this.motor.setZeroPowerBehavior(behaviour);
    }

    public void setMode(DcMotor.RunMode mode) {
        this.motor.setMode(mode);
    }
}
