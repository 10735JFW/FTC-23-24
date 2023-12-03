package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.java_websocket.server.WebSocketServer;

import java.net.InetSocketAddress;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class BasicCentricDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        String host = "192.168.43.1";
        int port = 8887;

//        WebSocketServer server = new SimpleServer(new InetSocketAddress(host, port));
//        server.start();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double gear = 1.00;
        waitForStart();
        int counter = 0;
        String oldPos= " ";
        int position = 30;

        while (!isStopRequested()) {
            drive.intakePower(gamepad2.right_trigger-gamepad2.left_trigger);
            String pos;
            pos = drive.armPower(position);
            if (gamepad2.left_stick_y<-0.5){
                position-=20;
                if(position < 30) {
                    position = 30;
                }
                pos = drive.armPower(position);
            }else if (gamepad2.left_stick_y>0.5) {
                position+=20;
                if(position > 550) {
                    position = 550;
                }
                pos = drive.armPower(position);
            }else {
                pos = drive.armPower(position);
            }
            if(!(oldPos.equals(pos))) {
                counter++;
//                server.broadcast(counter+", " + pos);
            }
            oldPos = pos;
            drive.clawPower(-gamepad2.right_stick_y);

            Pose2d poseEstimate = drive.getPoseEstimate();

            if(gamepad1.back){drive.resetIMU();}
            if(gamepad1.y){gear = 1.00;}
            if(gamepad1.x){gear = 0.55;}
            if(gamepad1.a){gear = 0.30;}

            Vector2d input = new Vector2d(
                    (gamepad1.left_stick_y*gear),
                    (gamepad1.left_stick_x*gear)

            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            (gamepad1.right_stick_x*gear)
                    )
            );

            drive.update();
            telemetry.addData("gear", gear);
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Left", drive.getleftfront());
            telemetry.addData("Right", drive.getrightfront());

            telemetry.update();
        }
    }
}

