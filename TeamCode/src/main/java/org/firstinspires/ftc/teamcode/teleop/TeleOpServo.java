package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedMotor;
import org.firstinspires.ftc.teamcode.hardware.common.OptimisedServo;

@Config
@TeleOp(name = "TeleOpServo", group = "Centerstage")
public class TeleOpServo extends LinearOpMode {

    public static volatile double ROTATION_POS = 0.5;
    public static volatile double PIXEL_LEFT_POS = 0.5;
    public static volatile double PIXEL_RIGHT_POS = 0.5;

    public static volatile double ANGLE_POS = 0.5;
    public static volatile double PIXEL_COVER_POS = 0.2;

    public static volatile double ARM_ANGLE_LEFT_POS = 0.5;
    public static volatile double CLAW_ANGLE_POS = 0.5;

    // claw
    private final OptimisedServo rotation = new OptimisedServo();
    private final OptimisedServo pixelLeft = new OptimisedServo();
    private final OptimisedServo pixelRight = new OptimisedServo();

    // intake
    private final OptimisedServo angle = new OptimisedServo();
    private final OptimisedServo pixelCover = new OptimisedServo();

    // v4b
    private final OptimisedServo armAngleLeft = new OptimisedServo();
    private final OptimisedServo armAngleRight = new OptimisedServo();
    private final OptimisedServo clawAngle = new OptimisedServo();

    @Override
    public void runOpMode() throws InterruptedException {
        rotation.setName("clawRotation", hardwareMap);
        pixelLeft.setName("clawPixelLeft", hardwareMap);
        pixelRight.setName("clawPixelRight", hardwareMap);

        angle.setName("intakeAngle", hardwareMap);
		pixelCover.setName("intakePixelCover", hardwareMap);

        armAngleLeft.setName("armAngleLeft", hardwareMap);
        armAngleRight.setName("armAngleRight", hardwareMap);
        clawAngle.setName("clawAngle", hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            rotation.setPosition(ROTATION_POS);
            pixelLeft.setPosition(PIXEL_LEFT_POS);
            pixelRight.setPosition(PIXEL_RIGHT_POS);

            angle.setPosition(ANGLE_POS);
            pixelCover.setPosition(PIXEL_COVER_POS);

            armAngleLeft.setPosition(ARM_ANGLE_LEFT_POS);
            armAngleRight.setPosition(1 - ARM_ANGLE_LEFT_POS);
            clawAngle.setPosition(CLAW_ANGLE_POS);
        }
    }

}
