package org.firstinspires.ftc.teamcode.hardware.subsystem.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class V4BConstants {

//    public static double ARM_ANGLE_DEPOSIT  = 0.22;
//    public static double ARM_ANGLE_TRANSFER = 0.75;
//
//    public static double CLAW_ANGLE_DEPOSIT = 0.8;
//    public static double CLAW_ANGLE_TRANSFER = 0.03;

    public static double ARM_ANGLE_DEPOSIT  = 0.22;
    public static double ARM_ANGLE_WAIT_FOR_COVER_RAISE = 0.66;
    public static double ARM_ANGLE_WAIT_ON_COVER = 0.72;
    public static double ARM_ANGLE_TRANSFER = 0.77;

    public static double CLAW_ANGLE_DEPOSIT = 0.8;
    public static double CLAW_ANGLE_WAIT_FOR_COVER_RAISE = 0.03;
    public static double CLAW_ANGLE_WAIT_ON_COVER = 0.03;
    public static double CLAW_ANGLE_TRANSFER = 0.03;

    public static volatile double ARM_MAX_ACC = 100;
    public static volatile double ARM_MAX_DEC = 2;
    public static volatile double ARM_MAX_VEL = 0.8;

    public static double CLAW_MAX_ACC = 100;
    public static double CLAW_MAX_DEC = 3;
    public static double CLAW_MAX_VEL = 2;

}
