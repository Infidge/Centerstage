package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystem.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystem.V4B;

public class Robot {

	private static Robot instance = null;

	public final Drivetrain drivetrain;
	public final Intake intake;
	public final Claw claw;
	public final Lift lift;
	public final V4B v4b;

	private Robot()	{
		drivetrain = new Drivetrain();
		intake = new Intake();
		claw = new Claw();
		lift = new Lift();
		v4b = new V4B();
	}

	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}

	public void init(HardwareMap hwMap) {
		drivetrain.init(hwMap);
		intake.init(hwMap);
		claw.init(hwMap);
		lift.init(hwMap);
		v4b.init(hwMap);
	}

	public void update(Telemetry telemetry) {
		intake.update();
		claw.update();
		lift.update();
		v4b.update(telemetry);
	}

}
