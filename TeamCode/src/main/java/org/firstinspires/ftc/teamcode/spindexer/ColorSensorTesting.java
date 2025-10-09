package org.firstinspires.ftc.teamcode.spindexer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@TeleOp(name = "ColorSensorTesting")
public class ColorSensorTesting extends OpMode {
    ColorSensor colorSensor;
    ButtonToggle buttonToggle;
    double r;
    double g;
    double b;
    double alpha;
    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    buttonToggle = new ButtonToggle();
    }

    @Override
    public void loop() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        if(buttonToggle.update(gamepad1.dpad_right)){
            r = colorSensor.red();
            g = colorSensor.green();
            b = colorSensor.blue();
            alpha = colorSensor.alpha();
        }

        telemetry.addData("r",r);
        telemetry.addData("g",g);
        telemetry.addData("b",b);
        telemetry.addData("a",alpha);
        telemetry.update();

    }
}
