package org.firstinspires.ftc.teamcode.tests.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.colors.ColorNormalizer;
import org.firstinspires.ftc.teamcode.colors.ColorSensorBuffer;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name = "ColorSensorTesting")
public class ColorSensorTesting extends OpMode {

    // Left sensor values
    int lR, lG, lB, lA;
    double lNormR, lNormG, lNormB;
    String lDetected = "No reading yet";
    String lDetectedBuffer = "No buffered reading yet";

    // Right sensor values
    int rR, rG, rB, rA;
    double rNormR, rNormG, rNormB;
    String rDetected = "No reading yet";
    String rDetectedBuffer = "No buffered reading yet";

    // Hardware
    ColorSensor leftSensor;
    ColorSensor rightSensor;

    // Button toggle for sampling
    ButtonToggle buttonToggle;

    // Buffers for each sensor
    ColorSensorBuffer lBufferRed, lBufferGreen, lBufferBlue;
    ColorSensorBuffer rBufferRed, rBufferGreen, rBufferBlue;

    // Normalizers
    ColorNormalizer lNorm;
    ColorNormalizer rNorm;

    // Green ball thresholds
    double greenRedMin = 0.05, greenRedMax = 0.40;
    double greenGreenMin = 0.645, greenGreenMax = 0.93;
    double greenBlueMin = 0.44, greenBlueMax = 0.75;

    // Purple ball thresholds
    double purpleRedMin = 0.28, purpleRedMax = 0.53;
    double purpleGreenMin = 0.30, purpleGreenMax = 0.73;
    double purpleBlueMin = 0.5875, purpleBlueMax = 0.93;

    @Override
    public void init() {

        leftSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColorLeft);
        rightSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColorRight);

        leftSensor.enableLed(true);
        rightSensor.enableLed(true);

        lNorm = new ColorNormalizer(0, 0, 0);
        rNorm = new ColorNormalizer(0, 0, 0);

        // Rolling buffers
        lBufferRed = new ColorSensorBuffer();
        lBufferGreen = new ColorSensorBuffer();
        lBufferBlue = new ColorSensorBuffer();

        rBufferRed = new ColorSensorBuffer();
        rBufferGreen = new ColorSensorBuffer();
        rBufferBlue = new ColorSensorBuffer();

        buttonToggle = new ButtonToggle();

        // Setup telemetry only once
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        if (buttonToggle.update(gamepad1.dpad_up)) {

            // LEFT SENSOR
            lR = leftSensor.red();
            lG = leftSensor.green();
            lB = leftSensor.blue();
            lA = leftSensor.alpha();

            lNorm.red = lR;
            lNorm.green = lG;
            lNorm.blue = lB;

            lNormR = lNorm.normalizeRed();
            lNormG = lNorm.normalizeGreen();
            lNormB = lNorm.normalizeBlue();

            lBufferRed.add(lNormR);
            lBufferGreen.add(lNormG);
            lBufferBlue.add(lNormB);

            lDetected = detectBallColor(lNormR, lNormG, lNormB);
            lDetectedBuffer = detectBallColor(
                    lBufferRed.getAverage(),
                    lBufferGreen.getAverage(),
                    lBufferBlue.getAverage()
            );

            // RIGHT SENSOR
            rR = rightSensor.red();
            rG = rightSensor.green();
            rB = rightSensor.blue();
            rA = rightSensor.alpha();

            rNorm.red = rR;
            rNorm.green = rG;
            rNorm.blue = rB;

            rNormR = rNorm.normalizeRed();
            rNormG = rNorm.normalizeGreen();
            rNormB = rNorm.normalizeBlue();

            rBufferRed.add(rNormR);
            rBufferGreen.add(rNormG);
            rBufferBlue.add(rNormB);

            rDetected = detectBallColor(rNormR, rNormG, rNormB);
            rDetectedBuffer = detectBallColor(
                    rBufferRed.getAverage(),
                    rBufferGreen.getAverage(),
                    rBufferBlue.getAverage()
            );
        }

        // TELEMETRY
        telemetry.addLine("=== LEFT Color Sensor ===");
        telemetry.addData("Raw R/G/B", "%d / %d / %d", lR, lG, lB);
        telemetry.addData("Norm R/G/B", "%.3f / %.3f / %.3f", lNormR, lNormG, lNormB);
        telemetry.addData("Avg R/G/B", "%.3f / %.3f / %.3f",
                lBufferRed.getAverage(), lBufferGreen.getAverage(), lBufferBlue.getAverage());
        telemetry.addData("Detected", lDetected);
        telemetry.addData("Detected (Buffered)", lDetectedBuffer);

        telemetry.addLine("=== RIGHT Color Sensor ===");
        telemetry.addData("Raw R/G/B", "%d / %d / %d", rR, rG, rB);
        telemetry.addData("Norm R/G/B", "%.3f / %.3f / %.3f", rNormR, rNormG, rNormB);
        telemetry.addData("Avg R/G/B", "%.3f / %.3f / %.3f",
                rBufferRed.getAverage(), rBufferGreen.getAverage(), rBufferBlue.getAverage());
        telemetry.addData("Detected", rDetected);
        telemetry.addData("Detected (Buffered)", rDetectedBuffer);

        telemetry.update();
    }

    private String detectBallColor(double r, double g, double b) {
        if (r >= greenRedMin && r <= greenRedMax &&
                g >= greenGreenMin && g <= greenGreenMax &&
                b >= greenBlueMin && b <= greenBlueMax)
            return "Green Ball";

        if (r >= purpleRedMin && r <= purpleRedMax &&
                g >= purpleGreenMin && g <= purpleGreenMax &&
                b >= purpleBlueMin && b <= purpleBlueMax)
            return "Purple Ball";

        return "Unknown Color";
    }
}
