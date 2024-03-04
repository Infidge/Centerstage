package org.firstinspires.ftc.teamcode.hardware.subsystem.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class V4BConstants {

    public static double DISTANCE = 0.75;

    public static double ARM_ANGLE_DEPOSIT  = 0.42;
    public static double ARM_ANGLE_TRANSFER = 0.955;

    public static double CLAW_ANGLE_DEPOSIT = 0.73;
    public static double CLAW_ANGLE_TRANSFER = 0.0;

    public static volatile double ARM_P = 0.0;
    public static volatile double ARM_D = 0.0;
    public static volatile double ARM_F = 0.0;

    public static volatile double ARM_MAX_ACC = 0.15;
    public static volatile double ARM_MAX_DEC = 0.15;
    public static volatile double ARM_MAX_VEL = 0.2;

}
