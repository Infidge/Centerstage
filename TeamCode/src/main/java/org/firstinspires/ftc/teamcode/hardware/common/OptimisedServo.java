package org.firstinspires.ftc.teamcode.hardware.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OptimisedServo {

    private Servo servo;

    private double lastPosition = 2.0;
    private static final double EPSILON = 0.003;

    public OptimisedServo() {}

    public void setName(String name, HardwareMap hwMap) {
        this.servo = hwMap.get(Servo.class, name);
    }

    public void setPosition(double position) {
        if (Math.abs(lastPosition - position) > EPSILON) {
            this.servo.setPosition(position);
            this.lastPosition = position;
        }
    }

    public void setPosition(double position, boolean inInit) {
        if (Math.abs(lastPosition - position) > EPSILON || inInit) {
            this.servo.setPosition(position);
            this.lastPosition = position;
        }
    }

    public double getPosition() {
        return this.lastPosition;
    }
}
