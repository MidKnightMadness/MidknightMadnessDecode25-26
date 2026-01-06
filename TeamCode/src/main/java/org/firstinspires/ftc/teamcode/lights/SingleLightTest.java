package org.firstinspires.ftc.teamcode.lights;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.game.BallColor;

public class SingleLightTest extends OpMode {
    LightThing light1;
    @Override
    public void init() {
        light1 = new LightThing(hardwareMap.get(Servo.class, "light1"));
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            light1.setColor(BallColor.PURPLE);
        }
        if(gamepad1.b){
            light1.setColor(BallColor.GREEN);
        }
        else{
            light1.setColor(BallColor.NONE);
        }
    }
}
