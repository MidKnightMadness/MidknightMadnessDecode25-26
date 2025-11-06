package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.ColorSensorThings.ColorNormalizer;


@TeleOp
public class BoxAlign extends OpMode {
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
    double lineLowerLimitRed = 0;
    double lineLowerLimitGreen = 0;
    double lineLowerLimitBlue = 0;
    double lineUpperLimitRed = 0;
    double lineUpperLimitGreen = 0;
    double lineUpperLimitBlue = 0;
    boolean inLimit;
    boolean leftLimit;
    boolean rightLimit;
    @Override
    public void init() {
        //make all the objects
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        drive = new WheelControlTemporaryMaybe(hardwareMap);
    }

    @Override
    public void loop() {
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

        //boolean stuff here because i don't want to put all of it in the if statement
        if (lnr>lineLowerLimitRed && lnr<lineUpperLimitRed && lng>lineLowerLimitGreen && lng<lineUpperLimitGreen && lnb>lineLowerLimitBlue && lnb<lineUpperLimitBlue && rnr>lineLowerLimitRed && rnr<lineUpperLimitRed && rng>lineLowerLimitGreen && rng<lineUpperLimitGreen && rnb>lineLowerLimitBlue && rnb<lineUpperLimitBlue){
            inLimit = true;
        }
        leftLimit = lnr > lineLowerLimitRed && lnr < lineUpperLimitRed && lng > lineLowerLimitGreen && lng < lineUpperLimitGreen && lnb > lineLowerLimitBlue && lnb < lineUpperLimitBlue;
        rightLimit = rnr > lineLowerLimitRed && rnr < lineUpperLimitRed && rng > lineLowerLimitGreen && rng < lineUpperLimitGreen && rnb > lineLowerLimitBlue && rnb < lineUpperLimitBlue;
        if (!inLimit) {
            drive.stop();
        }
        else if (!leftLimit && !rightLimit){
            drive.drive_relative(0.5, 0, 0, 1);
        }
        else if (!leftLimit && rightLimit){
            drive.drive_relative(0, 0, -0.5, 1);
        }
        else if (leftLimit && !rightLimit){
            drive.drive_relative(0, 0, 0.5, 1);
        }


        //
        //if colorsensorleft && colorsensor right senses:
        //  stop
        //elif colorsensorleft senses:
        //  while colorsensorleft senses:
        //      turn left
        //elif colorsensorright senses:
        //  while colorsensorright senses:
        //      turn right
        //else:
        //  go forwards

    }
}
/*
Goal: align to a box(red or blue(need to know which side we are on))
requirements:
- how big bot is
- 2 color sensors parallel

psueocode:
repeat this:
    go forward while doing colorsensor checks
    if one colorsensor activates, turn a bit in the direction that the color sensor is
    unloop if both colorsensor activates
go forward however much you need to get into the box
 */