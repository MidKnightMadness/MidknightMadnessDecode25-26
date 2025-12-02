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
//TODO: allow for two color sensors
    ColorSensorBuffer bufferRed;
    ColorSensorBuffer bufferGreen;
    ColorSensorBuffer bufferBlue;
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    ButtonToggle buttonToggle;

    int r, g, b, alpha;
    double normR, normG, normB;
    String detectedColor = "No reading yet";
    String detectedColorBuffer = "No buffered reading yet";

    // Thresholds for Green Ball (0.75" distance)
    double greenRedMin = 0.05;
    double greenRedMax = 0.40;
    double greenGreenMin = 0.645;
    double greenGreenMax = 0.93;
    double greenBlueMin = 0.44;
    double greenBlueMax = 0.75;

    // Thresholds for Purple Ball (0.75" distance)
    double purpleRedMin = 0.28;
    double purpleRedMax = 0.53;
    double purpleGreenMin = 0.30;
    double purpleGreenMax = 0.73;
    double purpleBlueMin = 0.5875;
    double purpleBlueMax = 0.93;
    ColorNormalizer norm;

    @Override
    public void init() {
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColorLeft);
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, ConfigNames.intakeColorRight);

        buttonToggle = new ButtonToggle();
        leftColorSensor.enableLed(true); //maybe not needed idk
        rightColorSensor.enableLed(true);
        norm = new ColorNormalizer(r, g, b);
        bufferRed = new ColorSensorBuffer();
        bufferGreen = new ColorSensorBuffer();
        bufferBlue = new ColorSensorBuffer();
    }

    @Override
    public void loop() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Check if dpad_up is pressed (with button toggle to avoid repeats)
        if (buttonToggle.update(gamepad1.dpad_up)) {
            // Read current color sensor values
            r = leftColorSensor.red();
            g = leftColorSensor.green();
            b = leftColorSensor.blue();
            alpha = leftColorSensor.alpha();
            norm.red = r;
            norm.green = g;
            norm.blue = b;
            //normalize colors
            normR = norm.normalizeRed();
            normG = norm.normalizeGreen();
            normB = norm.normalizeBlue();
            //Buffer
            bufferRed.addList(normR);
            bufferGreen.addList(normG);
            bufferBlue.addList(normB);
            // Detect color based on normalized RGB thresholds
            detectedColor = detectBallColor(normR, normG, normG);
            detectedColorBuffer = detectBallColor(bufferRed.num, bufferGreen.num, bufferBlue.num);
        }

        telemetry.addLine("LEFT color sensor");
        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Normalized Red", normR);
        telemetry.addData("Normalized Green", normG);
        telemetry.addData("Normalized Blue", normB);
        telemetry.addData("Buffered Red", bufferRed.num);
        telemetry.addData("Buffered Green", bufferGreen.num);
        telemetry.addData("Buffered Blue", bufferBlue.num);
        telemetry.addData("Alpha", alpha);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Detected Color(Buffer)", detectedColorBuffer);

        telemetry.addLine("RIGHT color sensor");
        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Normalized Red", normR);
        telemetry.addData("Normalized Green", normG);
        telemetry.addData("Normalized Blue", normB);
        telemetry.addData("Buffered Red", bufferRed.num);
        telemetry.addData("Buffered Green", bufferGreen.num);
        telemetry.addData("Buffered Blue", bufferBlue.num);
        telemetry.addData("Alpha", alpha);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Detected Color(Buffer)", detectedColorBuffer);
        telemetry.update();
    }

    private String detectBallColor(double r, double g, double b) {
        if (r >= greenRedMin && r <= greenRedMax &&
                g >= greenGreenMin && g <= greenGreenMax &&
                b >= greenBlueMin && b <= greenBlueMax) {
            return "Green Ball";
        } else if (r >= purpleRedMin && r <= purpleRedMax &&
                g >= purpleGreenMin && g <= purpleGreenMax &&
                b >= purpleBlueMin && b <= purpleBlueMax) {
            return "Purple Ball";
        } else {
            return "Unknown Color";
        }
    }
}