package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BoxAlign extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {

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