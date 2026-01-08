package org.firstinspires.ftc.teamcode.lights;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.game.BallColor;

public class LightThing{

    Servo lightControl;
    BallColor lastColor;
    public LightThing(Servo servo){
        lightControl = servo;
        lastColor = BallColor.NONE;
    }
    public void setColor(BallColor color){
        if(color.equals(lastColor)){
            return;
        }
        lastColor = color;
        if(color.equals(BallColor.PURPLE)){
            lightControl.setPosition(0.700);//i dont actually know the values ima just put random stuff until i can test
        }
        else if(color.equals(BallColor.GREEN)){
            lightControl.setPosition(0.500);
        }
        else{
            lightControl.setPosition(0);
        }
    }
}
//7 works as color