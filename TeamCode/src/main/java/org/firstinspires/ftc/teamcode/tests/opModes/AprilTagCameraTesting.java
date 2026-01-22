package org.firstinspires.ftc.teamcode.tests.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTag Camera Test")
public class AprilTagCameraTesting extends OpMode {

    // Telemetry telemetryM;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    static final int TAG_ID = 20;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, ConfigNames.arducam, telemetry);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TAG_ID);

        // camera fps and latency
        telemetry.addData("FPS", aprilTagWebcam.getFps());
        telemetry.addData("Latency (ms)", aprilTagWebcam.getLatencyMs());

        if (tag != null) {

            // position relative to camera (meters)
            double x = aprilTagWebcam.aprilTagXPos(tag);
            double z = aprilTagWebcam.aprilTagZPos(tag);

            // horizontal angle to apriltag
            double angleRad = Math.atan(x / z);
            double angleDeg = Math.toDegrees(angleRad);

            // straight line distance to april tag
            double distance = Math.hypot(x, z);

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("X Offset (m)", "%.3f", x);
            telemetry.addData("Z Distance (m)", "%.3f", z);
            telemetry.addData("Total Distance (m)", "%.3f", distance);
            telemetry.addData("Angle (deg)", "%.2f", angleDeg);

            //pose, yaw, pitch
            aprilTagWebcam.displayDetectionTelemetry(tag);

        } else {
            telemetry.addData("AprilTag " + TAG_ID, "Not Visible");
        }

        telemetry.update();
    }
}
