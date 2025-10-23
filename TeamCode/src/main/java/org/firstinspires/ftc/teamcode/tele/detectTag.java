package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "detectAprilTag", group = "Vision")
public class detectTag extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    //target tag id
    private final int targetTagId = 22;

    //Camera horizontal center in pixels
    private final double cameraCenterX = 640 / 2.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Create AprilTag Processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        //Create Vision Portal with webcam and AprilTag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(false)
                .build();

        telemetry.addLine("AprilTag Vision Ready. Press PLAY to start tracking.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("Detected Tags", detections.size());

            if (detections != null) {
                for (AprilTagDetection tag : detections) {
                    telemetry.addLine(String.format("Tag ID: %d", tag.id));
                    telemetry.addLine(String.format("Center (%.1f, %.1f)", tag.center.x, tag.center.y));

                    // TRACK TARGET TAG
                    if (tag.id == targetTagId) {
                        telemetry.addLine("Target tag detected! Turning to follow...");

                        //decide which way to turn
                        if (tag.center.x < cameraCenterX) {
                            //
                        } else {
                            //
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
}
