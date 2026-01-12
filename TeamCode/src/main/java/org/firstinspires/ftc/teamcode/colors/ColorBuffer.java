package org.firstinspires.ftc.teamcode.colors;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.hardware.BallDetector;

import java.util.ArrayList;

@Config
@Disabled
@Configurable
public class ColorBuffer {
    int purpleCount = 0;
    int greenCount = 0;
    int noneCount = 0;
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
        else{
            noneCount++;
        }


        if (loopRuns >= MAX_SIZE){
            BallColor majority = (purpleCount > greenCount && purpleCount > noneCount) ? BallColor.PURPLE : (greenCount > purpleCount && greenCount > noneCount) ? BallColor.GREEN : BallColor.NONE;
            prevColor = majority;

            //reset
            loopRuns = 0;
            purpleCount = 0;
            greenCount = 0;
            noneCount = 0;
        }
    }

    public int getGreenCount(){
        return greenCount;
    }

    public int getPurpleCount(){
        return purpleCount;
    }

    public int getNoneCount(){
        return noneCount;
    }
    public BallColor getColor(){
        return prevColor;
    }

}

