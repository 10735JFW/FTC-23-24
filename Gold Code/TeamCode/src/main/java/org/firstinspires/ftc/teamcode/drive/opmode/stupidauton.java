package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group = "test")
public class stupidauton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        drive.intakePower(0);
        drive.setWeightedDrivePower(new Pose2d(0.5,0, 0));
        timer.reset();
        while(timer.seconds() < .9) drive.update();
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        drive.intakePower(-1);
        while(timer.seconds() < 3) drive.update();
        drive.intakePower(0);
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.update();
    }
}