package org.firstinspires.ftc.teamcode.spindexer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.ultrasonics.ultrasonics;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@TeleOp(name = "ColorSensorTesting")
public class ColorSensorTesting extends OpMode {
    ColorSensor colorSensor;
    ButtonToggle buttonToggle;

    int r, g, b, alpha;

    String detectedColor = "No reading yet";

    // Thresholds for Green Ball (0.75" distance)
    int greenRedMin = 111;
    int greenRedMax = 113;
    int greenGreenMin = 404;
    int greenGreenMax = 408;
    int greenBlueMin = 309;
    int greenBlueMax = 311;

    // Thresholds for Purple Ball (0.75" distance)
    int purpleRedMin = 400;
    int purpleRedMax = 600;
    int purpleGreenMin = 400;
    int purpleGreenMax = 600;
    int purpleBlueMin = 900;
    int purpleBlueMax = 1000;
    ultrasonics ultrasonics;
    final double constantDistance = 5;
    final double distance1 = 1;
    final double distance2 = 1;
    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        buttonToggle = new ButtonToggle();
    }

    @Override
    public void loop() {
        double distance1 = ultrasonics.();



        if (buttonToggle.update(gamepad1.dpad_up)) {

            r = colorSensor.red();
            g = colorSensor.green();
            b = colorSensor.blue();
            alpha = colorSensor.alpha();

            // Detect color based on RGB thresholds
            detectedColor = detectBallColor(r, g, b);
        }

        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Alpha", alpha);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.update();
    }

    private String detectBallColor(int r, int g, int b) {
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
