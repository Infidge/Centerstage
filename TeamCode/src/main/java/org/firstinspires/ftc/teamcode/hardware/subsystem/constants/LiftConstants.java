package org.firstinspires.ftc.teamcode.hardware.subsystem.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LiftConstants {

    public static final double LOWER_POWER = -1.0;

    public static final int PIXEL_LEVEL_BASE = 100;
    public static final int PIXEL_LEVEL_INCREMENT = 200;

    public static final int PIXEL_LEVEL_MIN = 0;
    public static final int PIXEL_LEVEL_MAX = 10;

    public static double LIFT_P = 0.004;
    public static double LIFT_D = 0.001;
    public static double LIFT_F = 0.1;

}
