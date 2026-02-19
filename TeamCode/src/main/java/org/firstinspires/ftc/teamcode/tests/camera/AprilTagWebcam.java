package org.firstinspires.ftc.teamcode.tests.camera;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
@Config
public class AprilTagWebcam {
    public static int resolutionX = 1280;//640;
    public static int resolutionY = 800;//480;
    public static double fx = 905.83854;//549.651;
    public static double fy = 904.63725;//549.651;
    public static double cx = 609.96;//317.108;
    public static double cy = 386.20;//236.644;
    public static double posX = 0;
    public static double posY = 55;
    public static double posZ = 395;
    public static double yaw = 180;
    public static double pitch = 0;
    public static double roll = 90;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    public static int exposure = 255;
    public static int gain = 40;
    public static boolean changeExposure = true;

    private List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, String s, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setCameraPose(new Position(DistanceUnit.MM, posX, posY, posZ, 0), new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0))
                //calibrated values for monochrome one according to ftc discord
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, s));
        builder.setCameraResolution(new Size(resolutionX, resolutionY));

//        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);
        builder.addProcessor(aprilTagProcessor);
        if(changeExposure) {
            setManualExposure(exposure, gain);
        }// change depending on limelight vs other camera,
        // also resolution and device name based on config (or vice versa)

        visionPortal = builder.build();
    }
    public void init(HardwareMap hwMap, String s) {
       init(hwMap, s, null);
    }
    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
//            sleep(20);
            return (true);
//        } else {
//            return (false);
//        }
    }


    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    // telemetry

    public void displayDetectionTelemetry(org.firstinspires.ftc.vision.apriltag.AprilTagDetection detectedId) {

        if(telemetry == null){
            return;
        }
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
