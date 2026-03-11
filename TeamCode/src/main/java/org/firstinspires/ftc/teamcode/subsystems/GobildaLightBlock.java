package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.game.BallColor;

public class GobildaLightBlock {

    public enum Color{
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }
    public Servo lightControl;
    BallColor lastColor;
    public GobildaLightBlock(Servo servo){//consturctor
        lightControl = servo;
    }
    //for spindexer
    public void setColor(BallColor color){
//        if(color.equals(lastColor)){
//            return;
//        }
//        lastColor = color;
        if(color.equals(BallColor.PURPLE)){
            setColor(Color.VIOLET);
        } else if(color.equals(BallColor.GREEN)){
            setColor(Color.GREEN);
        } else if(color.equals(BallColor.UNKNOWN)){
            setColor(Color.ORANGE);
        } else{
            lightControl.setPosition(0);
        }
    }
    //anything else
    public void setColor(Color color){
        if(color.equals(Color.RED)){
            lightControl.setPosition(0.277);
        } else if(color.equals(Color.ORANGE)){
            lightControl.setPosition(0.333);
        } else if(color.equals(Color.YELLOW)){
            lightControl.setPosition(0.388);
        } else if(color.equals(Color.SAGE)){
            lightControl.setPosition(0.444);
        } else if(color.equals(Color.GREEN)){
            lightControl.setPosition(0.500);
        } else if(color.equals(Color.AZURE)){
            lightControl.setPosition(0.555);
        } else if(color.equals(Color.BLUE)){
            lightControl.setPosition(0.611);
        } else if(color.equals(Color.INDIGO)){
            lightControl.setPosition(0.666);
        } else if(color.equals(Color.VIOLET)){
            lightControl.setPosition(0.722);
        } else if(color.equals(Color.WHITE)){
            lightControl.setPosition(1.0);
        }
    }
}