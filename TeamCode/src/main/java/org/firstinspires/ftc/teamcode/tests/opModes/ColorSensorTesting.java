package org.firstinspires.ftc.teamcode.tests.opModes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.colors.ColorNormalizer;
import org.firstinspires.ftc.teamcode.colors.ColorSensorBuffer;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
@TeleOp(name = "ColorSensorTesting")
public class ColorSensorTesting extends OpMode {

    // Left sensor values
    float lR, lG, lB, lA;
    String lDetected = "No reading yet";
    String lDetectedBuffer = "No buffered reading yet";

    // Right sensor values
    float rR, rG, rB, rA;
    String rDetected = "No reading yet";
    String rDetectedBuffer = "No buffered reading yet";

    // Hardware
    RevColorSensorV3 leftSensor;
    RevColorSensorV3 rightSensor;

    // Button toggle for sampling
    ButtonToggle buttonToggle;

    // Buffers for each sensor
    ColorSensorBuffer lBufferRed, lBufferGreen, lBufferBlue;
    ColorSensorBuffer rBufferRed, rBufferGreen, rBufferBlue;

    // Normalizers
//    ColorNormalizer lNorm;
//    ColorNormalizer rNorm;

    // Green ball thresholds
    public static float greenHMin = 100f,  greenHMax = 180f;
    public static float greenSMin = 0.0f, greenSMax = 1f;
    public static float greenVMin = 0.0f, greenVMax = 1f;

    // Purple ball thresholds
    public static float purpleHMin = 0f, purpleHMax = 60f;
    public static float purpleSMin = 0f, purpleSMax = 1f;
    public static float purpleVMin = 0f, purpleVMax = 1f;
    float[] colorLeft = new float[]{0, 0, 0};
    float[] colorRight = new float[]{0, 0, 0};

    double leftDistance;
    double rightDistance;
    @Override
    public void init() {

        leftSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColorLeft);
        rightSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColorRight);


        leftSensor.enableLed(true);
        rightSensor.enableLed(true);

//        lNorm = new ColorNormalizer(0, 0, 0);
//        rNorm = new ColorNormalizer(0, 0, 0);

        // Rolling buffers
        lBufferRed = new ColorSensorBuffer();
        lBufferGreen = new ColorSensorBuffer();
        lBufferBlue = new ColorSensorBuffer();

        rBufferRed = new ColorSensorBuffer();
        rBufferGreen = new ColorSensorBuffer();
        rBufferBlue = new ColorSensorBuffer();

        buttonToggle = new ButtonToggle();

        // Setup telemetry only once
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        if (buttonToggle.update(gamepad1.dpad_up)) {

            // LEFT SENSOR
            lR = leftSensor.red();
            lG = leftSensor.green();
            lB = leftSensor.blue();
            lA = leftSensor.alpha();

            leftDistance = leftSensor.getDistance(DistanceUnit.INCH);
            colorLeft = new float[3];

            RGBToHSV(
                    leftSensor.red(),
                    leftSensor.green(),
                    leftSensor.blue(),
                    colorLeft
            );

            lBufferRed.add(colorLeft[0]);
            lBufferGreen.add(colorLeft[1]);
            lBufferBlue.add(colorLeft[2]);

            lDetected = detectBallColor(colorLeft[0], colorLeft[1], colorLeft[2]);
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
            rightDistance = rightSensor.getDistance(DistanceUnit.INCH);

            colorRight = new float[3];

            RGBToHSV(
                    rightSensor.red(),
                    rightSensor.green(),
                    rightSensor.blue(),
                    colorRight
            );

            rBufferRed.add(colorRight[0]);
            rBufferGreen.add(colorRight[1]);
            rBufferBlue.add(colorRight[2]);

            rDetected = detectBallColor(colorRight[0], colorRight[1], colorRight[2]);

            rDetectedBuffer = detectBallColor(
                    rBufferRed.getAverage(),
                    rBufferGreen.getAverage(),
                    rBufferBlue.getAverage()
            );
        }

        // TELEMETRY
        telemetry.addLine("=== LEFT Color Sensor ===");
        telemetry.addData("Raw R/G/B", "%f / %f / %f", lR, lG, lB);
        telemetry.addData("HSV", "%f / %f / %f",
                colorLeft[0],colorLeft[1], colorLeft[2]);
        telemetry.addData("Detected", lDetected);
        telemetry.addData("Detected (Buffered)", lDetectedBuffer);
        telemetry.addData("Distance", leftDistance);

        telemetry.addLine("=== RIGHT Color Sensor ===");
        telemetry.addData("Raw R/G/B", "%f / %f / %f", rR, rG, rB);
        telemetry.addData("HSV", "%f / %f / %f",
                colorRight[0],colorRight[1], colorRight[2]);
        telemetry.addData("Detected", rDetected);
        telemetry.addData("Detected (Buffered)", rDetectedBuffer);
        telemetry.addData("Distance", rightDistance);

        telemetry.update();
    }
    public static void RGBToHSV(int red, int green, int blue, float[] hsv) {
        float r = red / 255f;
        float g = green / 255f;
        float b = blue / 255f;

        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float h, s, v = max;

        // Hue
        if (delta == 0) {
            h = 0;
        } else if (max == r) {
            h = 60 * (((g - b) / delta) % 6);
        } else if (max == g) {
            h = 60 * (((b - r) / delta) + 2);
        } else { // max == b
            h = 60 * (((r - g) / delta) + 4);
        }
        if (h < 0) h += 360;

        // Saturation
        s = (max == 0) ? 0 : (delta / max);

        hsv[0] = h;
        hsv[1] = s;
        hsv[2] = v;
    }

    private String detectBallColor(double h, double s, double v) {
        if (h >= greenHMin && h <= greenHMax &&
                s >= greenSMin && s <= greenSMax &&
                v >= greenVMin && v <= greenVMax)
            return "Green Ball";

        if (h >= purpleHMin && h <= purpleHMax &&
                s >= purpleSMin && s <= purpleSMax &&
                v >= purpleVMin && v <= purpleVMax)
            return "Purple Ball";

        return "Unknown Color";
    }
}
