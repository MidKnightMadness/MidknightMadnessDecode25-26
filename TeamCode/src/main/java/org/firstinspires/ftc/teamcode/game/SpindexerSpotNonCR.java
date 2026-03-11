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
            (double)180/439,
            (double)360/439,
            Double.NaN
    ),
    SPOT1(
            1,
            (double)120/439,
            (double)300/439,
            Double.NaN,
            Double.NaN
    ),
    SPOT2(
            2,
            (double)240/439,
            (double)420/439,
            Double.NaN,
            (double)60/439
    ),
    SPOT3(
            3,
            (double)360/439,
            (double)60/439,
            Double.NaN,
            Double.NaN
    );

    public static int numSpots(){
        return 3;
    }


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
        if(!Double.isNaN(intakePosition1)){
            list.add(intakePosition1);
        }
        if(!Double.isNaN(intakePosition2)){
            list.add(intakePosition2);
        }
        return list;
    }
    public ArrayList<Double> getOutakePositions(){
        ArrayList<Double> list = new ArrayList<>();
        if(!Double.isNaN(outakePosition1)){
            list.add(outakePosition1);
        }
        if(!Double.isNaN(outakePosition2)){
            list.add(outakePosition2);
        }
        return list;
    }

    public double getIntakePositionSolo(){
       return intakePosition1;
    }
    public double getOuttakePositionSolo(){
        return outakePosition1;
    }

}
