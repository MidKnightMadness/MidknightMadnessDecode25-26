package org.firstinspires.ftc.teamcode.tests.color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
@Disabled
@TeleOp(name = "ColorSensorTesting")
public class ColorSensorTesting extends OpMode {

    // Left sensor values
    float lR, lG, lB, lA;
    String lDetected = "No reading yet";

    // Right sensor values
    float rR, rG, rB, rA;
    String rDetected = "No reading yet";

    // Bottom sensor values
    float bR, bG, bB, bA;
    String bDetected = "No reading yet";

    // Hardware
    RevColorSensorV3 leftSensor;
    RevColorSensorV3 rightSensor;
    RevColorSensorV3 bottomSensor;
    DistanceSensor distSensor;

    // Button toggle for sampling
    ButtonToggle buttonToggle;

//    // Buffers for each sensor
//    ColorSensorBuffer hBufferLeft, sBufferLeft, vBufferLeft;
//    ColorSensorBuffer hBufferRight, sBufferRight, vBufferRight;

    // Green ball thresholds
    public static float greenHMin = 150f,  greenHMax = 162f;
    public static float greenSMin = 0.5f, greenSMax = 1f;
    public static float greenVMin = 0f, greenVMax = 1f;

    // Purple ball thresholds
    public static float purpleHMin = 200f, purpleHMax = 235f;
    public static float purpleSMin = 0.33f, purpleSMax = 1f;
    public static float purpleVMin = 0f, purpleVMax = 1f;
    float[] colorLeft = new float[]{0, 0, 0};
    float[] colorRight = new float[]{0, 0, 0};
    float[] colorBottom = new float[]{0, 0, 0};

    double leftDistance;
    double rightDistance;
    double bottomDistance;

    @Override
    public void init() {

        leftSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColor1);
        rightSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColor2);
        bottomSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.bottomColor);
        distSensor = hardwareMap.get(DistanceSensor.class, ConfigNames.intakeDist1);

        leftSensor.enableLed(true);
        rightSensor.enableLed(true);
        bottomSensor.enableLed(true);


        // Rolling buffers
//        hBufferLeft = new ColorSensorBuffer();
//        sBufferLeft = new ColorSensorBuffer();
//        vBufferLeft = new ColorSensorBuffer();
//
//        hBufferRight = new ColorSensorBuffer();
//        sBufferRight = new ColorSensorBuffer();
//        vBufferRight = new ColorSensorBuffer();

        buttonToggle = new ButtonToggle();

    }

    @Override
    public void loop() {

        if (buttonToggle.update(gamepad1.dpad_up)) {
            getNormalizedRGBAs();

            //updates distance
            leftDistance = leftSensor.getDistance(DistanceUnit.CM);
            rightDistance = rightSensor.getDistance(DistanceUnit.CM);
            bottomDistance = rightSensor.getDistance(DistanceUnit.CM);


            colorLeft = new float[3];
            colorRight = new float[3];
            colorBottom = new float[3];


            RGBToHSV(
                    lR,  lG, lB, colorLeft
            );

            RGBToHSV(
                    rR,  rG,  rB, colorRight
            );
            RGBToHSV(
                    bR,  bG,  bB, colorBottom
            );



            lDetected = detectBallColor(colorLeft[0], colorLeft[1], colorLeft[2]);
            rDetected = detectBallColor(colorRight[0], colorRight[1], colorRight[2]);
            bDetected = detectBallColor(colorBottom[0], colorBottom[1], colorBottom[2]);

        }

        // TELEMETRY
        updateTelemetry();
    }

    public void getNormalizedRGBAs(){
        NormalizedRGBA leftNormValues = leftSensor.getNormalizedColors();
        lR = leftNormValues.red;
        lG = leftNormValues.green;
        lB = leftNormValues.blue;
        lA = leftNormValues.alpha;

        NormalizedRGBA rightNormValues = rightSensor.getNormalizedColors();
        rR = rightNormValues.red;
        rG = rightNormValues.green;
        rB = rightNormValues.blue;
        rA = rightNormValues.alpha;

        NormalizedRGBA bottomNormValues = bottomSensor.getNormalizedColors();
        bR = bottomNormValues.red;
        bG = bottomNormValues.green;
        bB = bottomNormValues.blue;
        bA = bottomNormValues.alpha;
    }

    public static void RGBToHSV(float red, float green, float blue, float[] hsv) {
        float r = red;// /255
        float g = green;// /255
        float b = blue;// /255

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

    //TODO: NEED TO test whether hsv works better or the normalized colors
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

    public void updateTelemetry(){
        telemetry.addData("Distance", distSensor.getDistance(DistanceUnit.INCH));
        telemetry.addLine("=== LEFT Color Sensor ===");
        telemetry.addData("Norm R/G/B", "%f / %f / %f", lR, lG, lB);
        telemetry.addData("HSV", "%f / %f / %f",
                colorLeft[0], colorLeft[1], colorLeft[2]);
        telemetry.addData("Detected", lDetected);
        telemetry.addData("Distance(CM", leftDistance);

        telemetry.addLine("=== RIGHT Color Sensor ===");
        telemetry.addData("Raw R/G/B", "%f / %f / %f", rR, rG, rB);
        telemetry.addData("HSV", "%f / %f / %f",
                colorRight[0], colorRight[1], colorRight[2]);
        telemetry.addData("Detected(CM)", rDetected);
        telemetry.addData("Distance(CM)", rightDistance);

        telemetry.addLine("=== BOTTOM Color Sensor ===");
        telemetry.addData("Raw R/G/B", "%f / %f / %f", bR, bG, bB);
        telemetry.addData("HSV", "%f / %f / %f",
                colorBottom[0], colorBottom[1], colorBottom[2]);
        telemetry.addData("Detected(CM)", bDetected);
        telemetry.addData("Distance(CM)", bottomDistance);

        telemetry.update();
    }
}

