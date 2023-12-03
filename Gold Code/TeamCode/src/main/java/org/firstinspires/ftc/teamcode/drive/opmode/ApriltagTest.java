package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AprilTagTestingDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(group = "test")
@Disabled
public class ApriltagTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor myAprilTagProcessor;
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal myVisionPortal;
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        AprilTagTestingDrive drive = new AprilTagTestingDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //run once

        while (!isStopRequested()) {
            //loop
            drive.update();
            telemetry.update();
        }

        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, false);
    }
}

