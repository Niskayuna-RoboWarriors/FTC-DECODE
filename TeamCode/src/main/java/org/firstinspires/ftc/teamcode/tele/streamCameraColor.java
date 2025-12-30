package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "streamCameraColor", group = "Vision")
public class streamCameraColor extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Target AprilTag ID
    private final int targetTagId = 22;

    // Camera horizontal center in pixels (assuming 640x480)
    private final double cameraCenterX = 640 / 2.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------
        // Initialize AprilTag Processor
        // -------------------------------
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // -------------------------------
        // Initialize VisionPortal
        // -------------------------------
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "colorCam"))
                .addProcessor(aprilTag)
                .enableLiveView(true) // This enables streaming to the Driver Station preview
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Faster than YUY2
                .setAutoStopLiveView(false)
                .build();

        telemetry.addLine("AprilTag Vision Ready. Press PLAY to start tracking.");
        telemetry.update();

        waitForStart();

        // -------------------------------
        // Main loop
        // -------------------------------
        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("Detected Tags", detections.size());

            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    telemetry.addLine(String.format("Tag ID: %d", tag.id));
                    telemetry.addLine(String.format("Center (%.1f, %.1f)", tag.center.x, tag.center.y));

                    // Target Tag logic
                    if (tag.id == targetTagId) {
                        telemetry.addLine("Target tag detected! Turning to follow...");

                        if (tag.center.x < cameraCenterX) {
                            telemetry.addLine("→ Turn Left");
                        } else {
                            telemetry.addLine("→ Turn Right");
                        }
                    }

                    // Pose info
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

        // Stop the stream
        visionPortal.close();
    }
}
