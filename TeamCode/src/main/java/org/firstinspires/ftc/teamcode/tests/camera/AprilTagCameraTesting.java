package org.firstinspires.ftc.teamcode.tests.camera;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AprilTag Field Localization Test")
@Config
@Configurable
public class AprilTagCameraTesting extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    AprilTagDetection tag;

    // field tag position in inches
    public static double TAG_20_X = 14;
    public static double TAG_20_Y = 130;
    public static double TAG_20_HEADING = 360 - 38.565; //53?

    public static double TAG_24_X = 130;
    public static double TAG_24_Y = 130;
    public static double TAG_24_HEADING = 38.565 + 180;

    // camera offset from robot center (inches)
    public static double CAMERA_FORWARD_OFFSET = 0;
    public static double CAMERA_LEFT_OFFSET = 0;

    double robotX;
    double robotY;
    double robotHeadingDeg;

    // exposure controls
    private VisionPortal portal;
    private ExposureControl exposureControl;
    private boolean exposureInitialized = false;

    private long currentExposureMs = 6;

    private boolean lastLeft = false;
    private boolean lastRight = false;
    // general offsets to try to fix extreme readings
    public static double generalOffsetX = 0;
    public static double generalOffsetY = 0;

    @Override
    public void init() {

        aprilTagWebcam.init(hardwareMap, ConfigNames.arducam, telemetry);
        portal = aprilTagWebcam.getVisionPortal();

        telemetry.addLine("Waiting for camera to start streaming...");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        if (!exposureInitialized &&
                portal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            exposureControl = portal.getCameraControl(ExposureControl.class);

            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(currentExposureMs, TimeUnit.MILLISECONDS);

            exposureInitialized = true;
        }

        telemetry.addData("Camera State", portal.getCameraState());
        telemetry.addData("Exposure Ready", exposureInitialized);
        telemetry.update();
    }

    @Override
    public void loop() {

        aprilTagWebcam.update();

        // exposure adjustment
        if (exposureInitialized) {

            boolean left = gamepad1.left_bumper;
            boolean right = gamepad1.right_bumper;

            if (right && !lastRight) {
                currentExposureMs++;
            }

            if (left && !lastLeft) {
                currentExposureMs--;
            }

            if (currentExposureMs < 1) currentExposureMs = 1;
            if (currentExposureMs > 50) currentExposureMs = 50;

            exposureControl.setExposure(currentExposureMs, TimeUnit.MILLISECONDS);

            lastLeft = left;
            lastRight = right;
        }

        tag = null;

        for (AprilTagDetection detection : aprilTagWebcam.getDetectedTags()) {
            if (detection.id == 20 || detection.id == 24) {
                tag = detection;
                break;
            }
        }

        telemetry.addData("Exposure (ms)", currentExposureMs);
        telemetry.addData("FPS", aprilTagWebcam.getFps());
        telemetry.addData("Latency (ms)", aprilTagWebcam.getLatencyMs());

        if (tag != null) {

            // convert cm to inches
            double relX = tag.ftcPose.y * 0.393701;   // forward becomes X
            double relY = -tag.ftcPose.x * 0.393701;  // left becomes -Y //change if it doesn't fix

            // get tag field data
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

            // get robot position
            robotX = tagFieldX - fieldOffsetX + generalOffsetX;
            robotY = tagFieldY - fieldOffsetY + generalOffsetY;

            // robot heading
            robotHeadingDeg = tagFieldHeadingDeg - tag.ftcPose.yaw;

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Robot X (in)", "%.2f", robotX);
            telemetry.addData("Robot Y (in)", "%.2f", robotY);
            telemetry.addData("Robot Heading (deg)", "%.2f", robotHeadingDeg);
            telemetry.addData("relX", relX);
            telemetry.addData("relY", relY);
            telemetry.addData("fieldOffsetX", fieldOffsetX);
            telemetry.addData("fieldOffsetY", fieldOffsetY);

        } else {
            telemetry.addLine("No Tag Visible");
        }

        telemetry.update();
    }
}
