package org.firstinspires.ftc.teamcode.spindexer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@TeleOp(name = "ColorSensorTesting")
public class ColorSensorTesting extends OpMode {
    ColorSensor colorSensor;
    ButtonToggle buttonToggle;

    int r, g, b, alpha;
    double normR, normG, normB;
    String detectedColor = "No reading yet";

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

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        buttonToggle = new ButtonToggle();
        colorSensor.enableLed(true); //maybe not needed idk
    }

    @Override
    public void loop() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Check if dpad_up is pressed (with button toggle to avoid repeats)
        if (buttonToggle.update(gamepad1.dpad_up)) {
            // Read current color sensor values
            r = colorSensor.red();
            g = colorSensor.green();
            b = colorSensor.blue();
            alpha = colorSensor.alpha();
            //normalize colors
            colorNormalizer norm = new colorNormalizer(r, g, b);
            normR = norm.normalizeRed();
            normG = norm.normalizeGreen();
            normB = norm.normalizeBlue();
            // Detect color based on normalized RGB thresholds
            detectedColor = detectBallColor(normR, normG, normB);
        }

        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Normalized Red", normR);
        telemetry.addData("Normalized Green", normG);
        telemetry.addData("Normalized Blue", normB);
        telemetry.addData("Alpha", alpha);
        telemetry.addData("Detected Color", detectedColor);
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
