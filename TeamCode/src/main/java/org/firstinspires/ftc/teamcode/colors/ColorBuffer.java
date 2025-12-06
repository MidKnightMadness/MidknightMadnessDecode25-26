package org.firstinspires.ftc.teamcode.colors;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.hardware.BallDetector;

import java.util.ArrayList;

@Config
@Configurable
public class ColorBuffer {
    public static int purpleCount = 0;
    public static int greenCount = 0;
    public static int minColorValue = 2;//need 4 at least to be definitively G/P
    public static final double MAX_SIZE = 5;
    public static int loopRuns = 0;

    BallColor prevColor;

    public ColorBuffer(){
    }

    public void add(BallColor color){
        loopRuns++;

        if(color == BallColor.GREEN){
            greenCount++;
        }
        else if(color == BallColor.PURPLE){
            purpleCount++;
        }

        if (loopRuns >= MAX_SIZE){
            loopRuns = 0;
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

