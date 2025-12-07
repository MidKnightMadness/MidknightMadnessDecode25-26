/*
package org.firstinspires.ftc.teamcode.Experiments;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSensorThings.ColorNormalizer;
import org.firstinspires.ftc.teamcode.ColorSensorThings.ColorSensorBuffer;


public class BoxAlignTest {
    ElapsedTime timer;
    static final double interval = 0.5;
    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;
    ColorNormalizer norm;
    WheelControlTemporaryMaybe drive;

    ColorSensorBuffer leftRedBuffer;
    ColorSensorBuffer leftGreenBuffer;
    ColorSensorBuffer leftBlueBuffer;

    ColorSensorBuffer rightRedBuffer;
    ColorSensorBuffer rightGreenBuffer;
    ColorSensorBuffer rightBlueBuffer;

    //red line values
    double redLineLowerLimitRed = 0.52;
    double redLineLowerLimitGreen = 0.61;
    double redLineLowerLimitBlue = 0.48;
    double redLineUpperLimitRed = 0.60;
    double redLineUpperLimitGreen = 0.675;
    double redLineUpperLimitBlue = 0.54;

    //blue line values


    double blueLineLowerLimitRed = 0.29;
    double blueLineLowerLimitGreen = 0.59;
    double blueLineLowerLimitBlue = 0.69;
    double blueLineUpperLimitRed = 0.35;
    double blueLineUpperLimitGreen = 0.63;
    double blueLineUpperLimitBlue = 0.75;


    boolean inLimit;
    boolean leftLimit;
    boolean rightLimit;

    Telemetry telemetry;
    //r and b for red and blue
    char color;

    double lnr;
    double lng;
    double lnb;
    double rnr;
    double rng;
    double rnb;
    public BoxAlignTest(HardwareMap hardwareMap, char color, Telemetry telemetry, WheelControlTemporaryMaybe drive) {
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorRight");
        this.drive = drive;
        this.color = color;
        this.telemetry = telemetry;
        timer = new ElapsedTime();
        leftRedBuffer = new ColorSensorBuffer();
        leftGreenBuffer = new ColorSensorBuffer();
        leftBlueBuffer = new ColorSensorBuffer();
        rightRedBuffer = new ColorSensorBuffer();
        rightBlueBuffer = new ColorSensorBuffer();
        rightGreenBuffer = new ColorSensorBuffer();
    }

    public void align() {
        //get the values
        double lr = colorSensorLeft.red();
        double lg = colorSensorLeft.green();
        double lb = colorSensorLeft.blue();
        double rr = colorSensorRight.red();
        double rg = colorSensorRight.green();
        double rb = colorSensorRight.blue();

        leftRedBuffer.addList(ColorNormalizer.normalizeRed(lr, lg, lb));
        leftGreenBuffer.addList(ColorNormalizer.normalizeGreen(lr, lg, lb));
        leftBlueBuffer.addList(ColorNormalizer.normalizeBlue(lr, lg, lb));

        rightRedBuffer.addList(ColorNormalizer.normalizeRed(rr, rg, rb));
        rightGreenBuffer.addList(ColorNormalizer.normalizeGreen(rr, rg, rb));
        rightBlueBuffer.addList(ColorNormalizer.normalizeBlue(rr, rg, rb));

        lnr = leftRedBuffer.num;
        lng = leftGreenBuffer.num;
        lnb = leftBlueBuffer.num;

        rnr = rightRedBuffer.num;
        rng = rightGreenBuffer.num;
        rnb = rightBlueBuffer.num;

        //this stuff is only for if you are blocking the method(put it in a while loop) if you aren't comment it out you wont have telemetry though
        /*
        telemetry.addData("Left N Red: ", lnr);
        telemetry.addData("Left N Green: ", lng);
        telemetry.addData("Left N Blue: ", lnb);

        telemetry.addData("Right N Red: ", rnr);
        telemetry.addData("Right N Green: ", rng);
        telemetry.addData("Right N Blue: ", rnb);

        if(timer.seconds() >= interval){
            telemetry.update();
            timer.reset();
        }

         */

/*

        if (color == 'r') {
            //boolean stuff here because i don't want to put all of it in the if statement
            if (lnr > redLineLowerLimitRed && lnr < redLineUpperLimitRed &&
                    lng > redLineLowerLimitGreen && lng < redLineUpperLimitGreen &&
                    lnb > redLineLowerLimitBlue && lnb < redLineUpperLimitBlue &&
                    rnr > redLineLowerLimitRed && rnr < redLineUpperLimitRed &&
                    rng > redLineLowerLimitGreen && rng < redLineUpperLimitGreen &&
                    rnb > redLineLowerLimitBlue && rnb < redLineUpperLimitBlue) {
                inLimit = true;
            }
            else{
                inLimit = false;
            }
            leftLimit = lnr > redLineLowerLimitRed && lnr < redLineUpperLimitRed &&
                    lng > redLineLowerLimitGreen && lng < redLineUpperLimitGreen &&
                    lnb > redLineLowerLimitBlue && lnb < redLineUpperLimitBlue;
            rightLimit = rnr > redLineLowerLimitRed && rnr < redLineUpperLimitRed &&
                    rng > redLineLowerLimitGreen && rng < redLineUpperLimitGreen &&
                    rnb > redLineLowerLimitBlue && rnb < redLineUpperLimitBlue;
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

        if (inLimit) {
            drive.stop();
        } else if (!leftLimit && !rightLimit) {
            //drive.drive_relative(0.25, 0, 0, 1); // small forward
            drive.setPowers(0.3, 0.3, 0.3, 0.3, 1);
        } else if (!leftLimit && rightLimit) {
            //drive.drive_relative(0, 0, -0.25, 1); // small turn right
            drive.setPowers(0.3, 0, 0.3, 0, 1);
        } else if (leftLimit && !rightLimit) {
            //drive.drive_relative(0, 0, 0.25, 1); // small turn left
            drive.setPowers(0, 0.3, 0, 0.3, 1);
        }
    }
}
*/