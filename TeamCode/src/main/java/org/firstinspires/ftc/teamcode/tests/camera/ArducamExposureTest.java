package org.firstinspires.ftc.teamcode.tests.camera;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Arducam Exposure Test", group = "Tests")
public class ArducamExposureTest extends LinearOpMode {

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 720));
        builder.enableLiveView(true);

        visionPortal = builder.build();

        telemetry.addLine("Waiting for camera to stream...");
        telemetry.update();


        while (!isStopRequested() &&
                visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        telemetry.addLine("Camera STREAMING");
        telemetry.update();

        // Get exposure control
        ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);

        GainControl gainControl =
                visionPortal.getCameraControl(GainControl.class);

        waitForStart();

        while (opModeIsActive()) {

            // Attempt to set Manual mode
            boolean manualSupported = exposureControl.setMode(ExposureControl.Mode.Manual);

            ExposureControl.Mode currentMode = exposureControl.getMode();

            // Try setting exposure
            exposureControl.setExposure(5, TimeUnit.MILLISECONDS);
            gainControl.setGain(5);

            long currentExposure =
                    exposureControl.getExposure(TimeUnit.MILLISECONDS);

            int currentGain = gainControl.getGain();

            telemetry.addData("Manual Mode Supported?", manualSupported);
            telemetry.addData("Current Exposure Mode", currentMode);
            telemetry.addData("Exposure (ms)", currentExposure);
            telemetry.addData("Gain", currentGain);
            telemetry.addData("FPS", visionPortal.getFps());
            telemetry.update();

            sleep(500);
        }

        visionPortal.close();
    }
}
