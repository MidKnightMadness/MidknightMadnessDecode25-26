package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.util.ColorSpace;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class SensorColor2 implements HardwareDevice {
    private double[] color;
    private ColorSpace colorSpace;
    private final ColorSensor colorSensor;

    /**
     * Constructs a color sensor with an explicit color space
     */
    public SensorColor2(ColorSensor colorSensor, ColorSpace colorSpace) {
        this.colorSpace = colorSpace;
        this.colorSensor = colorSensor;;
    }

    /**
     * Constructs a color sensor using the given hardware map and name with an explicit color space
     */
    public SensorColor2(HardwareMap hardwareMap, String name, ColorSpace colorSpace) {
        this(hardwareMap.get(ColorSensor.class, name), colorSpace);
    }

    /**
     * Constructs a color sensor, defaults to ARGB
     */
    public SensorColor2(ColorSensor colorSensor) {
        this(colorSensor, ColorSpace.RGBA);
    }

    /**
     * Constructs a color sensor using the given hardware map and name, defaults to ARGB
     */
    public SensorColor2(HardwareMap hardwareMap, String name) {
        this(hardwareMap.get(ColorSensor.class, name), ColorSpace.RGBA);
    }

    /**
     * Convert HSV value to an ARGB one. Includes alpha.
     *
     * @return an int representing the ARGB values
     */
    public int[] HSVtoARGB(int alpha, float[] hsv) {
        int color = Color.HSVToColor(alpha, hsv);
        return new int[]{Color.alpha(color), Color.red(color), Color.green(color), Color.blue(color)};
    }

    /**
     * Converts an RGB value to an HSV value. Provide the float[] to be used.
     */
    public float[] RGBtoHSV(int red, int green, int blue, float[] hsv) {
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
    }

    /**
     * Get all the ARGB values in an array from the sensor
     *
     * @return an int array representing ARGB
     */
    public int[] getARGB() {
        return new int[]{alpha(), red(), green(), blue()};
    }

    /**
     * Gets the alpha value from the sensor
     */
    public int alpha() {
        return colorSensor.alpha();
    }

    /**
     * Gets the red value from the sensor
     */
    public int red() {
        return colorSensor.red();
    }

    /**
     * Gets the green value from the sensor
     */
    public int green() {
        return colorSensor.green();
    }

    /**
     * Gets the blue value from the sensor
     */
    public int blue() {
        return colorSensor.blue();
    }

    @Override
    public void disable() {
        colorSensor.close();
    }

    @Override
    public String getDeviceType() {
        return "Color Sensor";
    }

}