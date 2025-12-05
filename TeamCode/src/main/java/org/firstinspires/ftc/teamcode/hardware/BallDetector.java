package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.colors.Threshold;
import java.util.Map;

@Config
@Configurable
public class BallDetector extends ColorDetector<BallColor> {
        private final ColorSensor colorSensor;

        public static Threshold[] greenThreshold = new Threshold[] {
                //normalized rgb out of magnitudes
                new Threshold(0.05f, 0.40f),
                new Threshold(0.645f, 0.93f),
                new Threshold(0.44f, 0.75f)
        };

        public static Threshold[] purpleThreshold = new Threshold[] {
                new Threshold(0.28f, 0.53f),
                        new Threshold(0.30f, 0.6f),
                        new Threshold(0.5875f, 09.3f)
        };
        public BallDetector(HardwareMap hardwareMap, String deviceName) {
                super(
                        Map.of(
                                BallColor.GREEN, greenThreshold,
                                BallColor.PURPLE, purpleThreshold
                        ),
                        BallColor.NONE
                );
                colorSensor = hardwareMap.get(ColorSensor.class, deviceName);
        }

        @Override
        public float[] readRawColor() {
                float[] color = new float[3];
                //dont read to hsv for now
//                color[0] = colorSensor.red();
//                color[1] = colorSensor.green();
//                color[2] = colorSensor.blue();
                Color.RGBToHSV(
                        colorSensor.red(),
                        colorSensor.green(),
                        colorSensor.blue(),
                        color
                );

//                double normRTotal = ColorNormalizer.normalizeRed(color[0], color[1], color[2]);
//                double normGTotal = ColorNormalizer.normalizeGreen(color[0], color[1], color[2]);
//                double normBTotal = ColorNormalizer.normalizeBlue(color[0], color[1], color[2]);
//
//                return new double[]{normRTotal, normGTotal, normBTotal};
                return color;
        }
}
