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
    public void runOpMode()
    {
        see = false;
        DriveConstants.spikePosition = -1;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));


        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (!isStopRequested())
        {
            telemetry.addData("Spike Position", DriveConstants.spikePosition);
            telemetry.addData("Did is reach the spot", reachSpot);
            if(DriveConstants.spikePosition !=-1) {
                if (DriveConstants.spikePosition == 0) {
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.seconds() < 5) {
                        double[] output = drive.moveInches(25, 30);
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
                    while (timer.seconds() < 4) {
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
                } else if (DriveConstants.spikePosition == 2) {
                    ElapsedTime timer = new ElapsedTime();
                    while (timer.seconds() < 4) {
                        double[] output = drive.moveInches(30, 25);
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
        //            if(DriveConstants.spikePosition !=-1){
//                if(DriveConstants.spikePosition==0){
//                    if(!reachSpot){
//                        reachSpot = drive.moveInches(30);
//                    }else {
//                        if(!startTurning){
//                            startTurning = drive.setAngle(225);
//                        }else {
//                            drive.intakePower(0.3);
//                        }
//                    }
//
//                }else if(DriveConstants.spikePosition==1) {
//                    if(!reachSpot){
//                        reachSpot = drive.moveInches(30);
//                    }else {
//                        drive.intakePower(0.3);
//                    }
//                }
//                else if(DriveConstants.spikePosition==2) {
//                    if(!reachSpot){
//                        reachSpot = drive.moveInches(30);
//                    }else {
//                        if(!startTurning){
//                            startTurning = drive.setAngle(135);
//                        }else {
//                            drive.intakePower(0.3);
//                        }
//                    }
//                }
        }
        /*
         * Send some stats to the telemetry
         */

        /*
         * NOTE: stopping the stream from the camera early (before the end of the OpMode
         * when it will be automatically stopped for you) *IS* supported. The "if" statement
         * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
         */
//            if(gamepad1.a)
//            {
//                webcam.stopStreaming();
//                //webcam.closeCameraDevice();
//            }
//            sleep(100);
        // }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
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

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

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