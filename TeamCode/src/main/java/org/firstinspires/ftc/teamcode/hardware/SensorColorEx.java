package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.SensorColor;

import org.firstinspires.ftc.teamcode.colors.ColorClassifier;
import org.firstinspires.ftc.teamcode.colors.ColorSpace;

public class SensorColorEx extends SensorColor {
    private final ColorSpace colorSpace;
    private final ColorSensor colorSensor;

    /**
     * Constructs a color sensor, defaults to ARGB
     */
    public SensorColorEx(ColorSensor colorSensor) {
        this(colorSensor, ColorSpace.RGBA);
    }

    /**
     * Constructs a color sensor using the given hardware map and name, defaults to ARGB
     */
    public SensorColorEx(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(ColorSensor.class, name), ColorSpace.RGBA);
    }

    /**
     * Constructs a color sensor with explicit color space
     */
    public SensorColorEx(ColorSensor colorSensor, ColorSpace colorSpace) {
        super(colorSensor);
        this.colorSensor = colorSensor;
        this.colorSpace = colorSpace;
    }

    /**
     * Constructs a color sensor using the given hardware map and name with explicit color space
     */
    public SensorColorEx(HardwareMap hardwareMap, String name, ColorSpace colorSpace) {
        this(hardwareMap.get(ColorSensor.class, name), colorSpace);
    }

    /**
     * Gets color in instance color space
     * @return Color array
     */
    public double[] getColor() {
        return getColor(colorSpace);
    }

    /**
     * Gets color in any color space
     * @return Color array
     */
    public double[] getColor(ColorSpace space) {
        return space.fromRgba(
                colorSensor.red(),
                colorSensor.green(),
                colorSensor.blue(),
                colorSensor.alpha()
        );
    }
}