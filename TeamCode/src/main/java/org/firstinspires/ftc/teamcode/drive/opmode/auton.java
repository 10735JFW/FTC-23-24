package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(group = "test")
public class auton extends LinearOpMode {
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

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested()) {
            telemetry.addData("spike", DriveConstants.spikePosition);
            if(DriveConstants.spikePosition !=-1) {
                if (DriveConstants.spikePosition == 0) {
                    if(timer.seconds() > 2) {
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    } else {
                        drive.setWeightedDrivePower(new Pose2d(0.5, 0, 0));
                    }
                } else if (DriveConstants.spikePosition == 1) {
                } else if (DriveConstants.spikePosition == 2) {
                }
            }
        }
        telemetry.update();
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            if(!see) {
                double[][] pixels = new double[input.rows()][input.cols()];
                for (int i = 0; i < input.rows(); i++) {
                    for (int j = 0; j < input.cols(); j++) {
                        double[] pixel = input.get(i, j);
                        double redness = Math.sqrt((pixel[0] - 255.0) * (pixel[0] - 255.0) + pixel[1] * pixel[1] + pixel[2] * pixel[2]);
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
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if(viewportPaused) {
                webcam.pauseViewport();
            }
            else {
                webcam.resumeViewport();
            }
        }
    }
}