package org.firstinspires.ftc.teamcode.tests.camera;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTag Field Localization Test")
@Config
@Configurable
public class AprilTagCameraTesting extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    AprilTagDetection tag;

    // === FIELD TAG POSITIONS (INCHES) ===
    public static double TAG_20_X = 0;
    public static double TAG_20_Y = 144;
    public static double TAG_20_HEADING = 0;   // degrees

    public static double TAG_24_X = 144;
    public static double TAG_24_Y = 144;
    public static double TAG_24_HEADING = 90;  // example

    // === Camera offset from robot center (inches) ===
    public static double CAMERA_FORWARD_OFFSET = 0;
    public static double CAMERA_LEFT_OFFSET = 0;

    double robotX;
    double robotY;
    double robotHeadingDeg;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, ConfigNames.arducam, telemetry);
        telemetry.addLine("Initialized");
    }

    @Override
    public void loop() {

        aprilTagWebcam.update();

        tag = null;

        // Grab first valid tag
        for (AprilTagDetection detection : aprilTagWebcam.getDetectedTags()) {
            if (detection.id == 20 || detection.id == 24) {
                tag = detection;
                break;
            }
        }

        telemetry.addData("FPS", aprilTagWebcam.getFps());
        telemetry.addData("Latency (ms)", aprilTagWebcam.getLatencyMs());

        if (tag != null) {

            // === Convert cm → inches ===
            double relX = tag.ftcPose.x * 0.393701; // left/right
            double relY = tag.ftcPose.y * 0.393701; // forward/back

            // === Get tag field data ===
            double tagFieldX = 0;
            double tagFieldY = 0;
            double tagFieldHeadingDeg = 0;

            if (tag.id == 20) {
                tagFieldX = TAG_20_X;
                tagFieldY = TAG_20_Y;
                tagFieldHeadingDeg = TAG_20_HEADING;
            } else if (tag.id == 24) {
                tagFieldX = TAG_24_X;
                tagFieldY = TAG_24_Y;
                tagFieldHeadingDeg = TAG_24_HEADING;
            }

            double theta = Math.toRadians(tagFieldHeadingDeg);

            // === Rotate relative pose into field coordinates ===
            double fieldOffsetX = relX * Math.cos(theta) - relY * Math.sin(theta);
            double fieldOffsetY = relX * Math.sin(theta) + relY * Math.cos(theta);

            // === Compute robot position ===
            robotX = tagFieldX - fieldOffsetX;
            robotY = tagFieldY - fieldOffsetY;

            // === Apply camera offset ===
            robotX -= CAMERA_FORWARD_OFFSET * Math.cos(theta);
            robotY -= CAMERA_LEFT_OFFSET * Math.sin(theta);

            // === Robot heading ===
            robotHeadingDeg = tagFieldHeadingDeg - tag.ftcPose.yaw;

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Robot X (in)", "%.2f", robotX);
            telemetry.addData("Robot Y (in)", "%.2f", robotY);
            telemetry.addData("Robot Heading (deg)", "%.2f", robotHeadingDeg);

        } else {
            telemetry.addLine("No Tag Visible");
        }

        telemetry.update();
    }
}
