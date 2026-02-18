package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.Angle;

@Config
@Configurable
public enum SpindexerSpot {
    SPOT0(
        0,
        Angle.fromDegrees(0),
        Angle.fromDegrees(180)
    ),
    SPOT1(
            1,
        Angle.fromDegrees(120),
        Angle.fromDegrees(300)
    ),
    SPOT2(
            2,
        Angle.fromDegrees(240),
        Angle.fromDegrees(60)
    );


    final double NUM_SPOTS = 3;
    Angle intakeAngle;
    Angle outtakeAngle;
    double intakePosition;
    double outakePosition;
    int index;
    SpindexerSpot(int index, Angle intakeAngle, Angle outtakeAngle) {
        this.index = index;
        this.intakeAngle = intakeAngle;
        this.outtakeAngle = outtakeAngle;
    }
    SpindexerSpot(int index, double intakeAngle, double outtakeAngle) {
        this.index = index;
        this.intakePosition = intakeAngle;
        this.outakePosition = outtakeAngle;
    }

    public static SpindexerSpot fromIndex(int index){
        for(SpindexerSpot spot : SpindexerSpot.values()){
            if(spot.index == index){
                return spot;
            }
        }
        return null;
    }

    public static SpindexerSpot[] convertFromindex(int[] index){
        SpindexerSpot[] arr = new SpindexerSpot[index.length];
        for(int i = 0; i < index.length; i++){
            arr[i] = fromIndex(index[i]);
        }
        return arr;
    }

    public Angle getSpotAngle(SpotType type){
        return (type == SpotType.OUTTAKE) ? outtakeAngle : intakeAngle;
    }

    public int getIndex(){
        return index;
    }

    public Angle getOuttakeAngle(){
        return outtakeAngle;
    }

    public Angle getIntakeAngle(){
        return intakeAngle;
    }

    public double getIntakePosition(){
        return intakePosition;
    }

    public double getOutakePosition(){
        return outakePosition;
    }


}
