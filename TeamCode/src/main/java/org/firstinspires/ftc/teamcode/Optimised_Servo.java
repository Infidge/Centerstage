package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Optimised_Servo {

    Servo servo;

    double lastPosition = 2.0;
    double epsilon = 0.01;

    public Optimised_Servo() {
    }

    public void setName(String name, HardwareMap hwMap) {
        this.servo = hwMap.get(Servo.class, name);
    }

    public void setPosition(Double position) {
        if (Math.abs(lastPosition - position) > epsilon) {
            this.servo.setPosition(position);
            this.lastPosition = position;
        }
    }

    public double getPosition() {
        return this.lastPosition;
    }
}
