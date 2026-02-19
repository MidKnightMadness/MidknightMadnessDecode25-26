package org.firstinspires.ftc.teamcode.tests.camera;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTag Position Test (No Odo)")
@Config
@Configurable
public class AprilTagCameraTesting extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    AprilTagDetection tag;

    // set to field values in inches
    public static double TAG_20_X = 0;
    public static double TAG_20_Y = 144;

    public static double TAG_24_X = 144;
    public static double TAG_24_Y = 144;

    // Camera offset from robot center (inches)
    public static double CAMERA_FORWARD_OFFSET = 0;   // + forward
    public static double CAMERA_LEFT_OFFSET = 0;      // + left

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

        tag = aprilTagWebcam.getTagBySpecificId(tag.id);


        telemetry.addData("FPS", aprilTagWebcam.getFps());
        telemetry.addData("Latency (ms)", aprilTagWebcam.getLatencyMs());

        if (tag != null) {

            // get camera relative pos

            double xCamMeters = tag.ftcPose.x;   // left/right
            double yCamMeters = tag.ftcPose.y;   // forward/back
            double yawDeg = tag.ftcPose.yaw;

            // Convert meters → inches
            double xCam = xCamMeters * 39.3701;
            double yCam = yCamMeters * 39.3701;

            // get tag field position

            double tagFieldX = 0;
            double tagFieldY = 0;

            if (tag.id == 20) {
                tagFieldX = TAG_20_X;
                tagFieldY = TAG_20_Y;
            } else if (tag.id == 24) {
                tagFieldX = TAG_24_X;
                tagFieldY = TAG_24_Y;
            }

            // find robot position

            // Robot position = Tag position − relative offset

            robotX = tagFieldX - yCam;
            robotY = tagFieldY - xCam;

            // Compensate camera offset
            robotX -= CAMERA_FORWARD_OFFSET;
            robotY -= CAMERA_LEFT_OFFSET;

            // Heading from tag yaw
            robotHeadingDeg = -yawDeg;

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
