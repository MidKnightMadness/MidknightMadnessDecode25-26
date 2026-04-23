package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Angle;

import java.util.ArrayList;

@Config
@Configurable
public enum SpindexerSpotNonCR {

    //-1 = not possible
    SPOT0(
            0,
            0 + SpindexerNonCR.SPINDEXER_OFFSET_DEGREES / SpindexerNonCR.totalDegrees,
            180d / SpindexerNonCR.totalDegrees,
            (double)360/439,
            Double.NaN
    ),
    SPOT1(
            1,
            (120 +  SpindexerNonCR.SPINDEXER_OFFSET_DEGREES) / SpindexerNonCR.totalDegrees,
            300d / SpindexerNonCR.totalDegrees,
            Double.NaN,
            Double.NaN
    ),
    SPOT2(
            2,
            (240d+  SpindexerNonCR.SPINDEXER_OFFSET_DEGREES) / SpindexerNonCR.totalDegrees,
            420d / SpindexerNonCR.totalDegrees,
            Double.NaN,
            60d / SpindexerNonCR.totalDegrees
    ),
    SPOT3(
            3,
            (360d+  SpindexerNonCR.SPINDEXER_OFFSET_DEGREES) / SpindexerNonCR.totalDegrees,
            60d / SpindexerNonCR.totalDegrees,
            Double.NaN,
            Double.NaN
    );

    public static int numSpots(){
        return 3;
    }



    public static int NUM_SPOTS = SpindexerNonCR.NUM_SPOTS;//in one revolution
    public static int MAX_SPOTS = SpindexerNonCR.TOTAL_SPOTS;
    double intakePosition1;
    double outakePosition1;
    double outakePosition2;
    double intakePosition2;
    int index;
    public static double OUTAKE_OFFSET_DEGREES = 20;
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

    public static int getMaxSpots(){
        return SPOT1.MAX_SPOTS;
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

    public double applyLayer(double position, int layer){
        return position + layer * 360 / SpindexerNonCR.totalDegrees;
    }

    public static double getPositionFromIndex(int globalIndex, SpotType type) {
        int baseIndex = globalIndex % NUM_SPOTS;//base spot
        int layer = globalIndex / NUM_SPOTS;//layering

        SpindexerSpotNonCR spot = fromIndex(baseIndex);

        double basePosition;
        if (type == SpotType.INTAKE) {
            basePosition = spot.getIntakePositionSolo();
        } else {
            basePosition = spot.getOuttakePositionSolo();
        }

        double offset = layer * 360.0 / SpindexerNonCR.totalDegrees;

        return Math.min(Math.max(basePosition + offset, 0), 1);
    }

}
