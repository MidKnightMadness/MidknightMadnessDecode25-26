package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.colors.Threshold;
import java.util.Map;

@Config
@Configurable
public class BallDetector extends ColorDetector<BallColor> {
        private final RevColorSensorV3 colorSensor;

        public static Threshold[] greenThreshold = new Threshold[] {
                //normalized rgb out of magnitudes
                new Threshold(100f, 180f),//hsv
                new Threshold(0f, 1f),
                new Threshold(0f, 1f)
        };

        public static Threshold[] purpleThreshold = new Threshold[] {
                new Threshold(0f, 60f),//hsv
                        new Threshold(0f, 1f),
                        new Threshold(0f, 1)
        };
        //consturctor
        public BallDetector(HardwareMap hardwareMap, String deviceName) {
                super(
                        Map.of(
                                BallColor.GREEN, greenThreshold,
                                BallColor.PURPLE, purpleThreshold
                        ),
                        BallColor.NONE
                );
                colorSensor = hardwareMap.get(RevColorSensorV3.class, deviceName);
                colorSensor.enableLed(true);
        }
        //converts rgb to hsv
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
        //gets the color
        @Override
        public float[] readRawColor() {
                float[] color = new float[3];
                //puts colors into a color array
                color[0] = colorSensor.red();
                color[1] = colorSensor.green();
                color[2] = colorSensor.blue();
                //converts all colors to hsv
                RGBToHSV(
                        colorSensor.red(),
                        colorSensor.green(),
                        colorSensor.blue(),
                        color
                );
                return color;
        }
        //gets distance
        public double getProximity() {
            return colorSensor.getDistance(DistanceUnit.INCH);
        }
}
