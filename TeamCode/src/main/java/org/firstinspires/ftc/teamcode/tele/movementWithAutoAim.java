package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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



@TeleOp(name="Movement With Auto Aim Tag", group="testing")
public class movementWithAutoAim extends LinearOpMode{
    IMU imu;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // change target tagid
    private final int targetTagId = 22;

    // Camera horizontal center in pixels
    private final double cameraCenterX = 640 / 2.0;




    public void runOpMode(){
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
            //vision






            //motion
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
        visionPortal.close();
    }




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
