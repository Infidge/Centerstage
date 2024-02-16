package org.firstinspires.ftc.teamcode.hardware.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotor;

public class Lift {

    private final OptimisedMotor liftMotor = new OptimisedMotor();

    public void init(HardwareMap hwMap) {
        liftMotor.setName("liftMotor", hwMap);
//        liftMotor.setDirection();
    }

}
