package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.spikePosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Timer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group = "test")
public class BlueUSBAuton extends LinearOpMode {
    OpenCvWebcam webcam;
    boolean see = false;
    int[] position = new int[2];
    double bestScore = 64*64;
    boolean reachSpot = false;
    boolean startTurning = false;
    @Override
    public void runOpMode() {
        see = false;
        DriveConstants.spikePosition = -1;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("Spike Position", DriveConstants.spikePosition);
            telemetry.addData("Did is reach the spot", reachSpot);
            if(DriveConstants.spikePosition !=-1) {
                drive.resetEncoder();
                if (DriveConstants.spikePosition == 0) {
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.seconds() < 5 && !isStopRequested()) {
                        double[] output = drive.moveInches(15, 35);
                        drive.update();
                        telemetry.addData("distace traveled left", output[0]);
                        telemetry.addData("distace traveled right", output[1]);
                        telemetry.addData("power left", output[2]);
                        telemetry.addData("power right", output[3]);
                        telemetry.addData("Spike Position", DriveConstants.spikePosition);
                    }
                    while (!isStopRequested()) {
                        telemetry.addData("Spike Position", DriveConstants.spikePosition);
                        drive.intakePower(0);
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        drive.update();
                    }
                } else if (DriveConstants.spikePosition == 2) {
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.seconds() < 4 && !isStopRequested()) {
                        double[] output = drive.moveInches(27, 27);
                        drive.update();
                        telemetry.addData("distace traveled left", output[0]);
                        telemetry.addData("distace traveled right", output[1]);
                        telemetry.addData("power left", output[2]);
                        telemetry.addData("power right", output[3]);
                        telemetry.addData("Spike Position", DriveConstants.spikePosition);
                    }
                    while (!isStopRequested()) {
                        telemetry.addData("Spike Position", DriveConstants.spikePosition);
                        drive.intakePower(0);
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        drive.update();
                    }
                } else if (DriveConstants.spikePosition == 1) {
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.seconds() < 4 && !isStopRequested()) {
                        double[] output = drive.moveInches(43, 15);
                        drive.update();
                        telemetry.addData("distace traveled left", output[0]);
                        telemetry.addData("distace traveled right", output[1]);
                        telemetry.addData("power left", output[2]);
                        telemetry.addData("power right", output[3]);
                        telemetry.addData("Spike Position", DriveConstants.spikePosition);
                    }
                    while (!isStopRequested()) {
                        telemetry.addData("Spike Position", DriveConstants.spikePosition);
                        drive.intakePower(0);
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        drive.update();
                    }
                }
            }
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            if(!see){
                double[][] pixels = new double[input.rows()][input.cols()];
                for (int i = 0; i < input.rows(); i++) {
                    for (int j = 0; j < input.cols(); j++) {
                        double[] pixel = input.get(i, j);
                        double redness = Math.sqrt((pixel[0]) * (pixel[0]) + pixel[1] * pixel[1] + (pixel[2]-255) * (pixel[2]-255));
                        if (redness > 300) {
                            pixels[i][j] = 1;
                        } else {
                            pixels[i][j] = redness / 300.0;
                        }
                    }
                }
                for (int i = 16; i < input.rows()-16; i++) {
                    for(int j = 16; j < input.cols()-16; j++) {
                        double score = 0;
                        for(int x = -16; x <  16; x++) {
                            for(int y = -16; y < 16; y++) {
                                score += pixels[i+x][j+y];
                            }
                        }
                        if(score < bestScore) {
                            bestScore = score;
                            position[0] = i;
                            position[1] = j;
                        }
                    }
                }
                see = true;
            }
            if(bestScore > 650) {
                telemetry.addData("Position", 0);
                DriveConstants.spikePosition = 0;
            }else {
                if(position[1] < input.cols()/2){
                    DriveConstants.spikePosition = 1;
                }else {
                    DriveConstants.spikePosition = 2;
                }
            }
            telemetry.addData("Position Y:", position[0]);
            telemetry.addData("Position X:", position[1]);
            telemetry.addData("bestScore:", bestScore);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
            Imgproc.rectangle(
                    input,
                    new Point(
                            (double)(position[1] - 32),
                            (double)(position[0] - 32)),
                    new Point(
                            (double)(position[1] + 32),
                            (double)(position[0] + 32)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}