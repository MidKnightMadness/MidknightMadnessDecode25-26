package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public enum ColorSpace {
    RGBA(4, -1, Imgproc.COLOR_RGBA2RGB),
    RGB(3, Imgproc.COLOR_RGBA2RGB, -1),
    HSV(3, Imgproc.COLOR_RGB2HSV, Imgproc.COLOR_HSV2RGB),
    Lab(3, Imgproc.COLOR_RGB2Lab, Imgproc.COLOR_Lab2RGB),
    Luv(3, Imgproc.COLOR_RGB2Luv, Imgproc.COLOR_Luv2RGB),
    XYZ(3, Imgproc.COLOR_RGB2XYZ, Imgproc.COLOR_XYZ2RGB),
    YCrCb(3, Imgproc.COLOR_RGB2YCrCb, Imgproc.COLOR_YCrCb2RGB),
    Gray(1, Imgproc.COLOR_RGB2GRAY, Imgproc.COLOR_GRAY2RGB);

    private final int length;
    private final int fromRgbConstant;
    private final int toRgbConstant;

    ColorSpace(int length, int fromRgbConstant, int toRgbConstant) {
        this.length = length;
        this.fromRgbConstant = fromRgbConstant;
        this.toRgbConstant = toRgbConstant;
    }

    public int getLength() {
        return length;
    }

    public int getFromRgbConstant() {
        return fromRgbConstant;
    }

    public int getToRgbConstant() {
        return fromRgbConstant;
    }

    /**
     * Function to convert a color array with a color conversion type.
     *
     * @param type  Color conversion type (get constant from Imgproc)
     * @param color Color array
     * @return Converted color array
     */
    public static double[] cvtColor(int type, double... color) {
        int cvType = CvType.makeType(CvType.CV_8U, color.length);
        Mat src = new Mat(1, 1, cvType);
        src.put(0, 0, color);
        Mat dst = new Mat();
        Imgproc.cvtColor(src, dst, type);
        return dst.get(0, 0);
    }

    /**
     * Throws exception if color array length does not fit color space
     * @param colorSpace Target color space
     * @param color Color array
     */
    private void checkLengthMismatch(ColorSpace colorSpace, double[] color) {
        if (colorSpace.length != color.length) {
            throw new IllegalArgumentException(
                    "Length of color array must be " + colorSpace.length +
                            " for " + colorSpace + ", but got: " + color.length
            );
        }
    }

    /**
     * Throws exception if color array length does not fit RGB or RGBA
     * @param color Color array
     */
    private void checkRgbLengthMismatch(double[] color) {
        if (ColorSpace.RGB.length != color.length
                && ColorSpace.RGBA.length != color.length) {
            throw new IllegalArgumentException(
                    "Length of color array must be 3 or 4 for RGB(A), but got: " + color.length
            );
        }
    }

    /**
     * Converts from RGBA to this color space
     * @param color A color in RGBA
     * @return Converted color in this color space
     */
    public double[] fromRgba(double... color) {
        checkLengthMismatch(ColorSpace.RGBA, color);
        if (this == ColorSpace.RGBA) return color;
        if (fromRgbConstant < 0) {
            throw new IllegalArgumentException("Unsupported conversion from RGBA to " + this);
        }
        return cvtColor(fromRgbConstant, color);
    }

    /**
     * Converts from RGB(A) to this color space. Note that RGB to RGBA is impossible.
     * @param color A color in RGB(A)
     * @return Converted color in this color space
     */
    public double[] fromRgb(double... color) {
        checkRgbLengthMismatch(color);
        if ((this == ColorSpace.RGBA && length == color.length) ||
                this == ColorSpace.RGB) return color;
        if (fromRgbConstant < 0) {
            throw new IllegalArgumentException("Unsupported conversion from RGB to " + this);
        }
        return cvtColor(fromRgbConstant, color);
    }

    /**
     * Converts from this color space to RGB
     * @param color A color in this color space
     * @return Converted color in RGB
     */
    public double[] toRgb(double... color) {
        checkLengthMismatch(this, color);
        if (this == ColorSpace.RGB) return color;
        if (toRgbConstant < 0) {
            throw new IllegalArgumentException("Unsupported conversion from RGB to " + this);
        }
        return cvtColor(toRgbConstant, color);
    }

    /**
     * Converts from this color space to another target color space
     * @param colorSpace Target color space
     * @param color Color array in this color space
     * @return Color array in target color space
     */
    public double[] to(ColorSpace colorSpace, double... color) {
        checkLengthMismatch(this, color);
        if (this == colorSpace) return color;
        return colorSpace.fromRgb(toRgb(color));
    }

    /**
     * Converts from source color space to this color space
     * @param colorSpace Source color space
     * @param color Color array in source color space
     * @return Color array in this color space
     */
    public double[] from(ColorSpace colorSpace, double... color) {
        checkLengthMismatch(colorSpace, color);
        if (this == colorSpace) return color;
        return fromRgb(colorSpace.toRgb(color));
    }
}
