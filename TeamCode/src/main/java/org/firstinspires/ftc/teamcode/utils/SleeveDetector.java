package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagDetectorJNI.TagFamily;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


public class SleeveDetector extends OpenCvPipeline {
    private final long tagdetectorPtr;
    private final Mat greyscale = new Mat();
    @NonNull private final Telemetry tm;

    public final OpenCvWebcam webcam;

    public SleeveDetector(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        super();

        tm = telemetry;
        tagdetectorPtr =
                AprilTagDetectorJNI.createApriltagDetector(TagFamily.TAG_36h11.string, 3.0F, 3);

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam((WebcamName) hardwareMap.get("webcam1"));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("CAMERA OPENING ERROR CODE: " + errorCode);
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 30.0);
        webcam.setPipeline(this);
    }

    protected void finalize() {
        if (tagdetectorPtr != 0) AprilTagDetectorJNI.releaseApriltagDetector(tagdetectorPtr);
        else System.out.println("AprilTagPipeline.finalize(): no tag detector address");
    }

    private int verdict = 0;

    public int getVerdict() {
        return verdict;
    }

    @NonNull
    public Mat processFrame(@NonNull Mat input) {
        try {
            Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_RGBA2GRAY);
            final var detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                    tagdetectorPtr,
                    greyscale,
                    0.166,
                    578.272,
                    578.272,
                    402.145,
                    221.506
            );
            if (detections.size() == 0) {
                tm.addLine("Not detected. FFFFFF");
            } else {
                final var detection = detections.get(0);
                verdict = detection.id;

                final var radius = detection.corners[1].x - detection.corners[0].x;
                Imgproc.circle(input, detection.center, (int) radius, GREEN, (int) (radius / 8.0));
                Imgproc.circle(
                        input,
                        detection.center,
                        (int) (radius / 20),
                        GREEN,
                        (int) (radius / 10)
                );
            }
        } catch (Exception e) {
            System.out.println("APRILTAG DETECTOR EXCEPTION:");
            e.printStackTrace();
        }

        tm.addData("Verdict", getVerdict());
        return input;
    }

    private static final Scalar GREEN = new Scalar(0.0, 255.0, 0.0, 255.0);
}
