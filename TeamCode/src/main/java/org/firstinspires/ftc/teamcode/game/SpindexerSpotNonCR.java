package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.Angle;

import java.util.ArrayList;

@Config
@Configurable
public enum SpindexerSpotNonCR {
    //-1 = not possible
    SPOT0(
            0,
            0,
            180/480,
            360/480,
            -1
    ),
    SPOT1(
            1,
            120/480,
            300/480,
            -1,
            -1
    ),
    SPOT2(
            2,
            240/480,
            420/480,
            -1,
            60/480
    );


    final double NUM_SPOTS = 3;
    double intakePosition1;
    double outakePosition1;
    double outakePosition2;
    double intakePosition2;
    int index;
    SpindexerSpotNonCR(int index, double intakePosition, double outakePosition, double intake2Position, double outake2Position){
        this.intakePosition1 = intakePosition;
        this.outakePosition1 = outakePosition;
        this.intakePosition2 = intake2Position;
        this.outakePosition2 = outake2Position;
        this.index = index;
    }

    public static SpindexerSpotNonCR fromIndex(int index){
        for(SpindexerSpotNonCR spot : SpindexerSpotNonCR.values()){
            if(spot.index == index){
                return spot;
            }
        }
        return null;
    }

    public static SpindexerSpotNonCR[] convertFromindex(int[] index){
        SpindexerSpotNonCR[] arr = new SpindexerSpotNonCR[index.length];
        for(int i = 0; i < index.length; i++){
            arr[i] = fromIndex(index[i]);
        }
        return arr;
    }

    public ArrayList<Double> getSpotPosition(SpotType type){
        if(type == SpotType.INTAKE){
            return getIntakePositions();
        } else{
            return getOutakePositions();
        }
    }


    public int getIndex(){
        return index;
    }

    public ArrayList<Double> getIntakePositions(){
        ArrayList<Double> list = new ArrayList<>();
        if(intakePosition1 != -1){
            list.add(intakePosition1);
        }
        if(intakePosition2 != -1){
            list.add(intakePosition2);
        }
        return list;
    }
    public ArrayList<Double> getOutakePositions(){
        ArrayList<Double> list = new ArrayList<>();
        if(outakePosition1 != -1){
            list.add(outakePosition1);
        }
        if(outakePosition2 != -1){
            list.add(outakePosition2);
        }
        return list;
    }

}
