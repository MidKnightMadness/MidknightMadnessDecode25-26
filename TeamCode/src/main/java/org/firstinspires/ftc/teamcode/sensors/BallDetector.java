package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Threshold;
import java.util.Map;

public class BallDetector extends ColorDetector<BallColor> {
        private final ColorSensor colorSensor;

        public BallDetector(HardwareMap hardwareMap, String deviceName) {
                super(
                        Map.of(
                                BallColor.GREEN, new Threshold[]{
                                        new Threshold(90f, 150f),   // H
                                        new Threshold(0.5f, 1f),    // S
                                        new Threshold(0.5f, 1f)     // V
                                },
                                BallColor.PURPLE, new Threshold[]{
                                        new Threshold(250f, 290f),
                                        new Threshold(0.5f, 1f),
                                        new Threshold(0.5f, 1f)
                                }
                        ),
                        BallColor.NONE
                );
                colorSensor = hardwareMap.get(ColorSensor.class, deviceName);
        }

        @Override
        public float[] readRawColor() {
                float[] color = new float[3];
                Color.RGBToHSV(
                        colorSensor.red(),
                        colorSensor.green(),
                        colorSensor.blue(),
                        color
                );
                return color;
        }
}
