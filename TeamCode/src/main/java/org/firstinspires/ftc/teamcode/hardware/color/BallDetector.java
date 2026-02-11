package org.firstinspires.ftc.teamcode.hardware.color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.game.BallColor;

@Config
@Configurable
public class BallDetector {
    private final RevColorSensorV3 colorSensor;

    public static Threshold[] greenThreshold = new Threshold[] {//color limits of green ball
            //normalized rgb out of magnitudes
            new Threshold(150f, 180f),//hsv
            new Threshold(0.5f, 1f),
            new Threshold(100f, 255f)
    };

    public static Threshold[] purpleThreshold = new Threshold[] {//color limits of purple ball
            new Threshold(200f, 235f),//hsv
            new Threshold(0.33f, 1f),
            new Threshold(100f, 255f)
    };
    public NormalizedRGBA normalizedRGBA;//define this
    public float[] hsv;
    public BallDetector(HardwareMap hardwareMap, String deviceName) {//constructor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, deviceName);
        colorSensor.enableLed(true);
    }

    public static void RGBToHSV(int red, int green, int blue, float[] hsv) {//formula to convert from rgb into hsv
        float r = red;
        float g = green;
        float b = blue;

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

    public float[] readRawColor() {//gets hsv
        float[] color = new float[3];

        normalizedRGBA = colorSensor.getNormalizedColors();
        color[0] = normalizedRGBA.red;
        color[1] = normalizedRGBA.green;
        color[2] = normalizedRGBA.blue;

        RGBToHSV(
                colorSensor.red(),
                colorSensor.green(),
                colorSensor.blue(),
                color
        );
        return color;
    }

    public BallColor getColor(){//gets ball color of measured ball(pruple or green)
        hsv = readRawColor();
        return detectBallColor(hsv[0], hsv[1], hsv[2]);
    }

    private BallColor detectBallColor(double h, double s, double v) {//gets ball color of ball inputted in
        if (h >= greenThreshold[0].low && h <= greenThreshold[0].high &&
                s >= greenThreshold[1].low && s <= greenThreshold[1].high &&
                v >= greenThreshold[2].low && v <= greenThreshold[2].high)
            return BallColor.GREEN;

        if (h >= purpleThreshold[0].low && h <= purpleThreshold[0].high &&
                s >= purpleThreshold[1].low && s <= purpleThreshold[1].high &&
                v >= purpleThreshold[2].low && v <= purpleThreshold[2].high)
            return BallColor.PURPLE;

        return BallColor.NONE;
    }
    public double getProximity() {
        return colorSensor.getDistance(DistanceUnit.INCH);
    }//gets distance
}
