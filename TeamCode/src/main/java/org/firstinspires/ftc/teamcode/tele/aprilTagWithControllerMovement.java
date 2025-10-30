package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "aprilTagWithControllerMovement", group = "Vision")
public class aprilTagWithControllerMovement extends LinearOpMode {
    IMU imu;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // chaneg tag ID
    private static final int TARGET_TAG_ID = 22;

    //Target distance from tag in inches
    private static final double TARGET_DISTANCE_INCHES = 12.0;
    private static final double DISTANCE_TOLERANCE_INCHES = 1.0;
    private static final double CENTER_TOLERANCE_INCHES = 1.0;

    @Override
    public void runOpMode() {
        //init hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera-1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(false)
                .build();

        //movemnetcode
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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        waitForStart();

        double pFrontLeftPower = 0;
        double pFrontRightPower = 0;
        double pBackLeftPower = 0;
        double pBackRightPower = 0;

        double motorEpsilon = 0.01;
        double joyStickZero = 0.01;
        double motorStopTryingThreshold = 0.05;

        boolean headingLock = true;
        boolean lockOnZeroAngularVelocity = false;
        double angularVelocityLockingThreshold = .3;
        double angularVelocityLockingThresholdDirection = 0;
        double headingLockDirection = 0;

        double headingPGain = 6.5 * 0.6;
        double headingDGain = 0.28;

        double loopRate = 0;

        while (opModeIsActive()){
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            timer.reset();
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            if (Math.abs(y) < joyStickZero) {
                y = 0;
            }
            if (Math.abs(x) < joyStickZero) {
                x = 0;
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            double yawAngularVelocity = angularVelocity.zRotationRate;
            double botHeading = orientation.getYaw(AngleUnit.RADIANS);
            if (botHeading < 0) {
                botHeading = 2 * Math.PI + botHeading;
            }
            if (Math.abs(rx) < joyStickZero) {
                rx = 0;
                if (!headingLock) {
                    headingLock = true;
                    lockOnZeroAngularVelocity = true;
                    angularVelocityLockingThresholdDirection = -Math.signum(yawAngularVelocity);
                    headingLockDirection = botHeading;
                }
                if (lockOnZeroAngularVelocity) {
                    if (Math.abs(yawAngularVelocity) < angularVelocityLockingThreshold
                            || Math.signum(yawAngularVelocity) == angularVelocityLockingThresholdDirection) {
                        headingLockDirection = botHeading;
                        lockOnZeroAngularVelocity = false;
                    }
                }
            } else {
                headingLock = false;
            }

            if (gamepad1.leftBumperWasPressed()) {
                headingLockDirection = 0;
            }
            if(gamepad1.squareWasPressed()){
                while (true){
                    if(gamepad1.squareWasPressed()){
                        break;
                    }
                    List<AprilTagDetection> detections = aprilTag.getDetections();

                    if (detections.size() > 0) {
                        AprilTagDetection target = null;

                        for (AprilTagDetection tag : detections) {
                            if (tag.metadata != null && tag.id == TARGET_TAG_ID) {
                                target = tag;
                                break;
                            }
                        }

                        if (target != null) {
                            double distanceInches = target.ftcPose.range;
                            double xOffsetInches = target.ftcPose.x; // left/right
                            double yawDegrees = target.ftcPose.yaw;

                            telemetry.addData("Detected Tag ID", target.id);
                            telemetry.addData("Distance (in)", "%.1f", distanceInches);
                            telemetry.addData("X Offset (in)", "%.1f", xOffsetInches);
                            telemetry.addData("Yaw (deg)", "%.1f", yawDegrees);

                            double forwardPower = 0;
                            double turnPower = 0;

                            // Maintain 12 inches distance
                            if (Math.abs(distanceInches - TARGET_DISTANCE_INCHES) > DISTANCE_TOLERANCE_INCHES) {
                                if (distanceInches > TARGET_DISTANCE_INCHES)
                                    forwardPower = 0.25; // move forward
                                else
                                    forwardPower = -0.25; // move backward
                            }

                            //Rotate to center (y-axis)
                            if (Math.abs(xOffsetInches) > CENTER_TOLERANCE_INCHES) {
                                if (xOffsetInches > 0)
                                    turnPower = 0.2; //tag is to the right --> turn right
                                else
                                    turnPower = -0.2; //tag is to the left --> turn left
                            }

                            drive(forwardPower, turnPower);
                        } else {
                            stopMotors();
                            telemetry.addLine("Target tag not in view");
                        }
                    } else {
                        stopMotors();
                        telemetry.addLine("No tags detected");
                    }

                    telemetry.update();
                }

            }

            if (headingLock) {
                double headingDifference = botHeading - headingLockDirection;
                if (headingDifference > Math.PI) {
                    headingDifference = headingDifference - 2 * Math.PI;
                }
                if (headingDifference < -Math.PI) {
                    headingDifference = 2 * Math.PI - headingDifference;
                }
                rx = headingPGain * headingDifference + headingDGain * yawAngularVelocity;
            }

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (Math.abs(pFrontLeftPower-frontLeftPower) > motorEpsilon) {
                if (Math.abs(frontLeftPower) < motorStopTryingThreshold) frontLeftPower = 0;
                frontLeft.setPower(frontLeftPower);
                pFrontLeftPower = frontLeftPower;
            }
            if (Math.abs(pBackLeftPower-backLeftPower) > motorEpsilon) {
                if (Math.abs(backLeftPower) < motorStopTryingThreshold) backLeftPower = 0;
                backLeft.setPower(backLeftPower);
                pBackLeftPower = backLeftPower;
            }
            if (Math.abs(pFrontRightPower-frontRightPower) > motorEpsilon) {
                if (Math.abs(frontRightPower) < motorStopTryingThreshold) frontRightPower = 0;
                frontRight.setPower(frontRightPower);
                pFrontRightPower = frontRightPower;
            }
            if (Math.abs(pBackRightPower-backRightPower) > motorEpsilon) {
                if (Math.abs(backRightPower) < motorStopTryingThreshold) backRightPower = 0;
                backRight.setPower(backRightPower);
                pBackRightPower = backRightPower;
            }

            telemetry.addData("Heading", botHeading);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Rad/Sec", angularVelocity.zRotationRate);
            loopRate = loopRate * 0.95 + 0.05/timer.time();
            telemetry.addData("Loop Rate", "%.2f Hz", loopRate);
            telemetry.update();
        }










        // inti AprilTag Processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        //create VisionPortal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera-1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(false)
                .build();

        telemetry.addLine("AprilTag Auto-Follow Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        //while (opModeIsActive()) {

        //}

        visionPortal.close();
    }



    // drive and turn functions
    private void drive(double forward, double turn) {
        double leftPower = forward + turn;
        double rightPower = forward - turn;

        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
