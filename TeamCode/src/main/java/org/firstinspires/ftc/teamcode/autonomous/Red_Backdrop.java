package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.RedDetection;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class Red_Backdrop extends LinearOpMode {

	enum AutoStages {
		detect,
		goToSpikeMark,
		goToBackdrop,
		goToPark
	}

	enum Randomisation {
		LEFT,
		MIDDLE,
		RIGHT
	}

	AutoStages currentState = AutoStages.detect;

	double startX = -90;
	double startY = 161.7;
	double startHeading = Math.toRadians(270);

	Randomisation randomisation = Randomisation.RIGHT;

	ElapsedTime detectionTime = new ElapsedTime();

	VisionPortal portal;
	RedDetection processor;

	TrajectorySequence goToSpikeMark;
	TrajectorySequence goToSpikeMarkLeft;
	TrajectorySequence goToSpikeMarkMid;
	TrajectorySequence goToSpikeMarkRight;

	@Override
	public void runOpMode() {
		Robot robot = Robot.getInstance();
		robot.init(hardwareMap);

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

		drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));

		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		processor = new RedDetection();
		portal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.setCameraResolution(new Size(1280, 720))
				.setCamera(BuiltinCameraDirection.BACK)
				.addProcessor(processor)
				.enableLiveView(true)
				.build();

		generatePaths(drive);

		int detectionCase = 1;

		while (opModeInInit() && !isStopRequested()) {
			detectionCase = processor.detection;

			telemetry.addData("Detection", processor.detection);
			telemetry.update();
		}

		waitForStart();

		while (opModeIsActive()) {
			robot.update(telemetry);
			drive.update();
			switch (currentState) {
				case detect:
					if (detectionCase == 1)
						randomisation = Randomisation.LEFT;
					else if (detectionCase == 2)
						randomisation = Randomisation.MIDDLE;
					else if (detectionCase == 3)
						randomisation = Randomisation.RIGHT;
					else randomisation = Randomisation.RIGHT;
					//chooseParkPath(drive);
					currentState = AutoStages.goToSpikeMark;
					break;
				case goToSpikeMark:

					break;
				case goToBackdrop:
					break;
				case goToPark:
					break;
			}
			break;
		}

	}

	public void generatePaths(SampleMecanumDrive drive)
	{
		goToSpikeMark = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
				.splineToLinearHeading(new Pose2d(30, -120, Math.toRadians(90.0)), Math.toRadians(90.0))
				.addDisplacementMarker( () -> {
					if (randomisation == Randomisation.LEFT)
						drive.followTrajectorySequenceAsync(goToSpikeMarkLeft);
					else if (randomisation == Randomisation.MIDDLE)
						drive.followTrajectorySequenceAsync(goToSpikeMarkMid);
					else
						drive.followTrajectorySequenceAsync(goToSpikeMarkRight);
				})
				.build();

		goToSpikeMarkLeft = drive.trajectorySequenceBuilder(goToSpikeMark.end())
				.setTangent(Math.toRadians(90.0))
				.splineTo(new Vector2d(20, -90), Math.toRadians(45.0))
				.addDisplacementMarker(() -> {

				})
				.build();

		goToSpikeMarkMid = drive.trajectorySequenceBuilder(goToSpikeMark.end())
				.setTangent(Math.toRadians(90.0))
				.splineTo(new Vector2d(30, -90), Math.toRadians(90.0))
				.build();

		goToSpikeMarkRight = drive.trajectorySequenceBuilder(goToSpikeMark.end())
				.setTangent(Math.toRadians(90.0))
				.splineTo(new Vector2d(40, -90), Math.toRadians(135.0))
				.build();



	}
}
