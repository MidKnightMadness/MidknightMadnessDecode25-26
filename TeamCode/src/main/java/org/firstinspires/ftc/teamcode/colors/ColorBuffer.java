package org.firstinspires.ftc.teamcode.colors;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.hardware.BallDetector;

import java.util.ArrayList;

public class ColorBuffer {
    public int purpleCount = 0;
    public int greenCount = 0;
    public int minColorValue = 4;//need 4 at least to be definitively G/P
    public final double MAX_SIZE = 5;
    public int loopRuns = 0;

    BallColor prevColor;

    public ColorBuffer(){
    }

    public void add(BallColor color){
        if(color != BallColor.NONE){
            loopRuns++;
        }
        if(color == BallColor.GREEN){
            greenCount++;
        }
        else if(color == BallColor.PURPLE){
            purpleCount++;
        }

        if (loopRuns >= MAX_SIZE){
            BallColor majority = (purpleCount > greenCount && purpleCount > minColorValue) ? BallColor.PURPLE : (greenCount > purpleCount && greenCount > minColorValue) ? BallColor.GREEN : BallColor.NONE;
            prevColor = majority;
            purpleCount = 0;
            greenCount = 0;
        }
    }
    public BallColor getColor(){
        return prevColor;
    }

}

