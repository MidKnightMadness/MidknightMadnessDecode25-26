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
    Servo lightControl;
    BallColor lastColor;
    double currPower = 0;

    public GobildaLightBlock(Servo servo){
        lightControl = servo;
        lastColor = BallColor.NONE;
    }

    //for spindexer
    public void setColor(BallColor color){
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
        double targetPower;
        switch (color){
            case RED:
                targetPower = 0.297;
            case ORANGE:
                targetPower = 0.333;
            case YELLOW:
                targetPower = 0.388;
            case SAGE:
                targetPower = 0.444;
            case GREEN:
                targetPower = 0.500;
            case AZURE:
                targetPower = 0.555;
            case BLUE:
                targetPower = 0.611;
            case INDIGO:
                targetPower = 0.666;
            case VIOLET:
                targetPower = 0.722;
            case WHITE:
                targetPower = 1.0;
            default:
                targetPower = 0;
        }
        if(targetPower != currPower){
            lightControl.setPosition(targetPower);
            currPower = targetPower;
        }

    }
}