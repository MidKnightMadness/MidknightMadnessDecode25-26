
package org.firstinspires.ftc.teamcode.Experiments;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ColorSensorThings.ColorNormalizer;


public class BoxAlign {
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;
    ColorNormalizer norm;
    WheelControlTemporaryMaybe drive;
    double lr, lg, lb;
    double rr, rg, rb;

    //normalized ones
    double lnr, lng, lnb;
    double rnr, rng, rnb;

    //put actual values here after testing using the tape

    //red line values
    double redLineLowerLimitRed = 0;
    double redLineLowerLimitGreen = 0;
    double redLineLowerLimitBlue = 0;
    double redLineUpperLimitRed = 0;
    double redLineUpperLimitGreen = 0;
    double redLineUpperLimitBlue = 0;

    //blue line values
    double blueLineLowerLimitRed = 0;
    double blueLineLowerLimitGreen = 0;
    double blueLineLowerLimitBlue = 0;
    double blueLineUpperLimitRed = 0;
    double blueLineUpperLimitGreen = 0;
    double blueLineUpperLimitBlue = 0;
    boolean inLimit;
    boolean leftLimit;
    boolean rightLimit;

    char color;
    public BoxAlign(HardwareMap hardwareMap, char color) {
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorRight");
        drive = new WheelControlTemporaryMaybe(hardwareMap);
        this.color = color;
    }


    public void align() {
        //get the values
        lr = colorSensorLeft.red();
        lg = colorSensorLeft.green();
        lb = colorSensorLeft.blue();
        rr = colorSensorRight.red();
        rg = colorSensorRight.green();
        rb = colorSensorRight.blue();

        lnr = ColorNormalizer.normalizeRed(lr, lg, lb);
        lng = ColorNormalizer.normalizeGreen(lr, lg, lb);
        lnb = ColorNormalizer.normalizeBlue(lr, lg, lb);

        rnr = ColorNormalizer.normalizeRed(rr, rg, rb);
        rng = ColorNormalizer.normalizeGreen(rr, rg, rb);
        rnb = ColorNormalizer.normalizeBlue(rr, rg, rb);

        if (color == 'r') {
            //boolean stuff here because i don't want to put all of it in the if statement
            if (lnr > redLineLowerLimitRed && lnr < redLineUpperLimitRed && lng > redLineLowerLimitGreen && lng < redLineUpperLimitGreen && lnb > redLineLowerLimitBlue && lnb < redLineUpperLimitBlue && rnr > redLineLowerLimitRed && rnr < redLineUpperLimitRed && rng > redLineLowerLimitGreen && rng < redLineUpperLimitGreen && rnb > redLineLowerLimitBlue && rnb < redLineUpperLimitBlue) {
                inLimit = true;
            }
            else{
                inLimit = false;
            }
            leftLimit = lnr > redLineLowerLimitRed && lnr < redLineUpperLimitRed && lng > redLineLowerLimitGreen && lng < redLineUpperLimitGreen && lnb > redLineLowerLimitBlue && lnb < redLineUpperLimitBlue;
            rightLimit = rnr > redLineLowerLimitRed && rnr < redLineUpperLimitRed && rng > redLineLowerLimitGreen && rng < redLineUpperLimitGreen && rnb > redLineLowerLimitBlue && rnb < redLineUpperLimitBlue;
        }
        if (color == 'b') {
            // boolean stuff here because i don't want to put all of it in the if statement
            if (lnr > blueLineLowerLimitRed && lnr < blueLineUpperLimitRed &&
                    lng > blueLineLowerLimitGreen && lng < blueLineUpperLimitGreen &&
                    lnb > blueLineLowerLimitBlue && lnb < blueLineUpperLimitBlue &&
                    rnr > blueLineLowerLimitRed && rnr < blueLineUpperLimitRed &&
                    rng > blueLineLowerLimitGreen && rng < blueLineUpperLimitGreen &&
                    rnb > blueLineLowerLimitBlue && rnb < blueLineUpperLimitBlue) {
                inLimit = true;
            }
            else{
                inLimit = false;
            }

            leftLimit = lnr > blueLineLowerLimitRed && lnr < blueLineUpperLimitRed &&
                    lng > blueLineLowerLimitGreen && lng < blueLineUpperLimitGreen &&
                    lnb > blueLineLowerLimitBlue && lnb < blueLineUpperLimitBlue;

            rightLimit = rnr > blueLineLowerLimitRed && rnr < blueLineUpperLimitRed &&
                    rng > blueLineLowerLimitGreen && rng < blueLineUpperLimitGreen &&
                    rnb > blueLineLowerLimitBlue && rnb < blueLineUpperLimitBlue;
        }

        if (!inLimit) {
            drive.stop();
        } else if (!leftLimit && !rightLimit) {
            drive.drive_relative(0.2, 0, 0, 1); // small forward
        } else if (!leftLimit && rightLimit) {
            drive.drive_relative(0, 0, -0.1, 1); // small turn left
        } else if (leftLimit && !rightLimit) {
            drive.drive_relative(0, 0, 0.1, 1); // small turn right
        } else {
            drive.stop();
        }
    }
}