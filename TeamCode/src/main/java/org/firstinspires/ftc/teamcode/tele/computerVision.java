package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import org.openftc.apriltag.AprilTagDetector;      // direct detector class (OpenFTC april tag wrapper)
import org.openftc.apriltag.AprilTagDetection;    // detection results

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

@TeleOp
public class AprilTagDirectTest extends LinearOpMode {
    OpenCvWebcam webcam;
    AprilTagDirectPipeline pipeline;

    // camera intrinsics (example values — calibrate for best pose accuracy)
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagSizeMeters = 0.166;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new AprilTagDirectPipeline(tagSizeMeters, fx, fy, cx, cy);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Init done - waiting");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Get the latest detections (thread-safe view)
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

            if (detections != null && detections.size() > 0) {
                telemetry.addData("Num tags", detections.size());
                for (AprilTagDetection d : detections) {
                    telemetry.addData("Tag ID", d.id);
                    // if pose is available on the detection you can also print translation/rotation fields:
                    // telemetry.addData("pose x", d.pose.x); etc.
                }
            } else {
                telemetry.addData("Num tags", 0);
            }

            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Pipeline that converts to gray and calls the AprilTagDetector directly.
     * Keeps a thread-safe copy of the last detections for the OpMode to read.
     */
    public static class AprilTagDirectPipeline extends OpenCvPipeline {
        private final AprilTagDetector detector;
        private final double tagSize, fx, fy, cx, cy;

        // thread-safe reference to last detections
        private volatile ArrayList<AprilTagDetection> latestDetections = new ArrayList<>();

        // temporary mats reused to avoid allocations
        private final Mat gray = new Mat();

        public AprilTagDirectPipeline(double tagSizeMeters, double fx, double fy, double cx, double cy) {
            this.tagSize = tagSizeMeters;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            // Create detector and set the tag family to detect.
            // "tag36h11" is the most common family used by most AprilTag generators.
            detector = new AprilTagDetector();
            detector.addFamily("tag36h11");   // add the family you need

            // If the wrapper supports setting camera intrinsics/pose estimation parameters,
            // set them here if required by the detector (some wrappers accept them at detect time).
            // For the OpenFTC wrapper, pose estimation fields are computed into the detection if
            // you pass tag size and intrinsics to the detector/detect call (see detect call below).
        }

        @Override
        public Mat processFrame(Mat input) {
            // convert to gray (AprilTag detectors expect grayscale)
            if (input.channels() == 3) {
                Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            } else {
                input.copyTo(gray);
            }

            // run detection. The exact detector.detect(...) signature varies by wrapper.
            // The OpenFTC java wrapper provides detect(gray, tagSize, fx, fy, cx, cy) typically.
            // We'll attempt to call that form — if your version differs, call the appropriate overload.
            AprilTagDetection[] detections = detector.detect(gray, tagSize, fx, fy, cx, cy);

            // store a thread-safe copy for the opmode
            ArrayList<AprilTagDetection> list = new ArrayList<>();
            if (detections != null) {
                for (AprilTagDetection d : detections) list.add(d);
            }
            latestDetections = list; // volatile write

            // Optionally annotate the frame (draw boxes/ids) - for simplicity return input unmodified.
            // If you want annotations, draw on `input` using OpenCV drawing functions.

            return input;
        }

        /** Returns the most recently detected tags (may be empty). */
        public ArrayList<AprilTagDetection> getLatestDetections() {
            return latestDetections;
        }

        @Override
        public void onViewportTapped() {
            // optional: react to user tapping the camera monitor
        }
    }
}