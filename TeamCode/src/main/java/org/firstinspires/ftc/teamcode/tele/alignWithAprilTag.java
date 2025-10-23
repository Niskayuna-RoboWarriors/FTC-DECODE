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

@TeleOp(name = "FTC SDK AprilTag Auto Follow", group = "Vision")
public class alignWithAprilTag extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Desired tag ID
    private static final int TARGET_TAG_ID = 22;

    // Target distance from tag in inches
    private static final double TARGET_DISTANCE_INCHES = 12.0;
    private static final double DISTANCE_TOLERANCE_INCHES = 1.0;
    private static final double CENTER_TOLERANCE_INCHES = 1.0;

    @Override
    public void runOpMode() {
        // --- Initialize hardware ---
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

        // --- Initialize AprilTag Processor ---
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // --- Create VisionPortal with webcam ---
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        telemetry.addLine("AprilTag Auto-Follow Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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

                    // --- Maintain 12 inches distance ---
                    if (Math.abs(distanceInches - TARGET_DISTANCE_INCHES) > DISTANCE_TOLERANCE_INCHES) {
                        if (distanceInches > TARGET_DISTANCE_INCHES)
                            forwardPower = 0.25; // move forward
                        else
                            forwardPower = -0.25; // move backward
                    }

                    // --- Rotate to center ---
                    if (Math.abs(xOffsetInches) > CENTER_TOLERANCE_INCHES) {
                        if (xOffsetInches > 0)
                            turnPower = 0.2; // tag is to the right → turn right
                        else
                            turnPower = -0.2; // tag is to the left → turn left
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

        visionPortal.close();
    }

    // --- Simple tank-like drive for forward + turning motion ---
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
