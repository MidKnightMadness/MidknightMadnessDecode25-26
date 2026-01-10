package org.firstinspires.ftc.teamcode.lights;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
public class lightsTest extends OpMode {
    LightThing light1;
    LightThing light2;
    LightThing light3;
    BallColor[] list;
    Spindexer spindexer;
    BallColor[] setA = {BallColor.GREEN, BallColor.GREEN, BallColor.PURPLE};
    BallColor[] setB = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    BallColor[] setX = {BallColor.GREEN, BallColor.GREEN, BallColor.GREEN};
    BallColor[] setY = {BallColor.NONE, BallColor.NONE, BallColor.NONE};
    @Override
    public void init() {
        light1 = new LightThing(hardwareMap.get(Servo.class, "light1"));
        light1 = new LightThing(hardwareMap.get(Servo.class, "light2"));
        light2 = new LightThing(hardwareMap.get(Servo.class, "light3"));
        spindexer = new Spindexer(hardwareMap, true, new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()){
            spindexer.setBallColors(setA);
        }
        if(gamepad1.bWasPressed()){
            spindexer.setBallColors(setB);
        }
        if(gamepad1.xWasPressed()){
            spindexer.setBallColors(setX);
        }
        if(gamepad1.yWasPressed()){
            spindexer.setBallColors(setY);
        }
        list = spindexer.getBallColors();
        if(list[0] == BallColor.GREEN){
            light1.setColor(BallColor.GREEN);
        }
        else if(list[0] == BallColor.PURPLE){
            light1.setColor(BallColor.PURPLE);
        }
        else{
            light1.setColor(BallColor.NONE);
        }
        if(list[1] == BallColor.GREEN){
            light2.setColor(BallColor.GREEN);
        }
        else if(list[1] == BallColor.PURPLE){
            light2.setColor(BallColor.PURPLE);
        }
        else{
            light2.setColor(BallColor.NONE);
        }
        if(list[2] == BallColor.GREEN){
            light3.setColor(BallColor.GREEN);
        }
        else if(list[2] == BallColor.PURPLE){
            light3.setColor(BallColor.PURPLE);
        }
        else{
            light3.setColor(BallColor.NONE);
        }
    }
}
