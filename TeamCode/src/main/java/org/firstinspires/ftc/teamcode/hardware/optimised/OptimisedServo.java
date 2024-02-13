package org.firstinspires.ftc.teamcode.hardware.optimised;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OptimisedServo {

    private Servo servo;

    private double lastPosition = 2.0;
    private static final double EPSILON = 0.01;

    public OptimisedServo() {}

    public void setName(String name, HardwareMap hwMap) {
        this.servo = hwMap.get(Servo.class, name);
    }

    public void setPosition(Double position) {
        if (Math.abs(lastPosition - position) > EPSILON) {
            this.servo.setPosition(position);
            this.lastPosition = position;
        }
    }

    public double getPosition() {
        return this.lastPosition;
    }
}
