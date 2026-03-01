package org.firstinspires.ftc.teamcode.tests.color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.hardware.SensorColorEx;
import org.firstinspires.ftc.teamcode.hardware.color.ColorClassifier;
import org.firstinspires.ftc.teamcode.hardware.color.ColorSpace;
import org.firstinspires.ftc.teamcode.hardware.color.Threshold;

import java.util.Map;

@TeleOp
@Disabled
@Configurable
public class SolversColorSensorTest extends OpMode {
    public static ColorSpace space = ColorSpace.HSV;
    ColorClassifier<BallColor> classifier;
    SensorColorEx colorSensor;
    TelemetryManager telemetryM;

    @Override
    public void init() {
         colorSensor = new SensorColorEx(hardwareMap, "colorSensor");
         colorSensor.setColorSpace(space);
         telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
         classifier = new ColorClassifier.Builder<BallColor>()
                 .add(
                         BallColor.GREEN,
                         new Threshold(55, 80),
                         new Threshold(50, 200),
                         new Threshold(3, 20)
                 )
                 .add(
                         BallColor.PURPLE,
                         new Threshold(0, 30),
                         new Threshold(50, 200),
                         new Threshold(3, 20)
                 )
                 .setDefault(BallColor.NONE)
                 .build();
    }

    @Override
    public void loop() {
        double[] color = colorSensor.getColor();
        telemetryM.addData("color value 0", color[0]);
        telemetryM.addData("color value 1", color[1]);
        telemetryM.addData("color value 2", color[2]);
        telemetryM.addData("classified", classifier.classify(color));
        telemetryM.update(telemetry);
    }
}
