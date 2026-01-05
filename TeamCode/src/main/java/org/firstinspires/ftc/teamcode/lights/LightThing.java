package org.firstinspires.ftc.teamcode.lights;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.game.BallColor;

public class LightThing{

    Servo lightControl;
    public LightThing(Servo servo){
        lightControl = servo;
        lightControl.setPosition(0);
    }
    public void setColor(BallColor color){
        if(color.equals(BallColor.PURPLE)){
            lightControl.setPosition(6);//i dont actually know the values ima just put random stuff until i can test
        }
        else if(color.equals(BallColor.GREEN)){
            lightControl.setPosition(7);
        }
        else{
            lightControl.setPosition(0);
        }
    }
}
