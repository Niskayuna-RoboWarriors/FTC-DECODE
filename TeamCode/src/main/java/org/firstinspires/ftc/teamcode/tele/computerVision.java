package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Rotating AprilTag Tracker", group = "Vision")
public class computerVision extends LinearOpMode {
    IMU imu;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // change target tagid
    private final int targetTagId = 22;

    // Camera horizontal center in pixels
    private final double cameraCenterX = 640 / 2.0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        //Create AprilTag Processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        //Create Vision Portal with webcam and AprilTag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera-1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(false)
                .build();

        telemetry.addLine("AprilTag Vision Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("Detected Tags", detections.size());

            if (detections != null) {// check for tag ids here
                for (AprilTagDetection tag : detections) {
                    telemetry.addLine(String.format("Tag ID: %d", tag.id));
                    telemetry.addLine(String.format("Center (%.1f, %.1f)", tag.center.x, tag.center.y));

                    //TRACK TARGET TAG
                    if (tag.id == targetTagId) {
                        telemetry.addLine("Target tag detected! Turning to follow...");

                        //choose which way to turn
                        if (tag.center.x < cameraCenterX) {
                            telemetry.addLine("Aligning Robot Left");
                            telemetry.addData("CameraCenter:", "", cameraCenterX);
                            telemetry.addData("TagCenter:", "", tag.center.x);
                            turnLeft(0.01, frontLeft, frontRight, backLeft, backRight);   // your custom rotation code goes here


                        } else {
                            telemetry.addLine("Aligning Robot Right");
                            telemetry.addData("CameraCenter:", "", cameraCenterX);
                            telemetry.addData("TagCenter:", "", tag.center.x);
                            turnRight(0.01, frontLeft, frontRight, backLeft, backRight);  // your custom rotation code goes here
                        }
                    }

                    // show pose info
                    if (tag.metadata != null) {
                        telemetry.addData("Distance (in)", "%.1f", tag.ftcPose.range);
                        telemetry.addData("Bearing (deg)", "%.1f", tag.ftcPose.bearing);
                        telemetry.addData("Yaw (deg)", "%.1f", tag.ftcPose.yaw);
                    }
                    telemetry.addLine("-----------------------");
                }
            }

            telemetry.update();
            sleep(50);
        }

        visionPortal.close();
    }

    //ROTATION
    private void turnLeft(double power, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    private void turnRight(double power, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    private void stopMotors(DcMotorEx frontLeft, DcMotorEx frontRight,
                            DcMotorEx backLeft, DcMotorEx backRight) {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

}
