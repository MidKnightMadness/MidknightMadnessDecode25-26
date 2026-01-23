package org.firstinspires.ftc.teamcode.tests.opModes;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, String s, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES) //change distance unit if its causing
                .build();                                           // problems

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, s));
        builder.setCameraResolution(new Size(1280, 720));
        builder.enableLiveView(true);
        builder.addProcessor(aprilTagProcessor); // change depending on limelight vs other camera,
        // also resolution and device name based on config (or vice versa)

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    // telemetry

    public void displayDetectionTelemetry(org.firstinspires.ftc.vision.apriltag.AprilTagDetection detectedId) {

        if (detectedId == null) {
            telemetry.addLine("Correct ID not detected");
            return;
        }

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format( //change this weird string thing if it's buggy (idk why its like this)
                    "\n==== (ID %d) %s",
                    detectedId.id, detectedId.metadata.name));

            telemetry.addLine(String.format(
                    "XYZ %6.1f %6.1f %6.1f (cm)",
                    detectedId.ftcPose.x,
                    detectedId.ftcPose.y,
                    detectedId.ftcPose.z));

            telemetry.addLine(String.format(
                    "PRY %6.1f %6.1f %6.1f (deg)",
                    detectedId.ftcPose.pitch,
                    detectedId.ftcPose.roll,
                    detectedId.ftcPose.yaw));

            telemetry.addLine(String.format(
                    "Range %.1f cm  Bearing %.1f°  Elev %.1f°",
                    detectedId.ftcPose.range,
                    detectedId.ftcPose.bearing,
                    detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format(
                    "\n==== (ID %d) Unknown", detectedId.id));

            telemetry.addLine(String.format(
                    "Center %6.0f %6.0f (pixels)",
                    detectedId.center.x,
                    detectedId.center.y));
        }
    }

    // pose

    public double aprilTagXPos(org.firstinspires.ftc.vision.apriltag.AprilTagDetection detectedId) {
        return detectedId.ftcPose.x;
    }

    public double aprilTagZPos(org.firstinspires.ftc.vision.apriltag.AprilTagDetection detectedId) {
        return detectedId.ftcPose.z;
    }

    public String aprilTagXOffsetPos(org.firstinspires.ftc.vision.apriltag.AprilTagDetection detectedId) {
        double bounds = 1.0;
        if (detectedId.ftcPose.x > bounds) {
            return "april tag is " + detectedId.ftcPose.x + " cm on right";
        } else if (detectedId.ftcPose.x < -bounds) {
            return "april tag is " + (-detectedId.ftcPose.x) + " cm on left";
        } else {
            return "roughly centered between (" + -bounds + ", " + bounds + ") cm";
        }
    }

    // april tag

    public org.firstinspires.ftc.vision.apriltag.AprilTagDetection getTagBySpecificId(int id) {
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    // camera fps + latency display

    public double getFps() {
        return visionPortal != null ? visionPortal.getFps() : 0.0;
    }

    // latency based on fps (not exact)
    public double getLatencyMs() {
        double fps = getFps();
        return fps > 0 ? (1000.0 / fps) : 0.0;
    }

    // shutdown/end step

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
