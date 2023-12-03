package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group = "drive")
public class FirstCustomSpline extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 0), 0)
                .splineTo(new Vector2d(24, 32), Math.PI/2)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(20)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(9-1, 40, Math.toRadians(270)))
                .build();
        // :]

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }
}
