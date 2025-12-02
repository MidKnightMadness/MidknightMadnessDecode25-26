package org.firstinspires.ftc.teamcode.game;

import org.firstinspires.ftc.teamcode.util.Angle;

public enum SpindexerSpot {
    SPOT0(
        0,
        Angle.fromDegrees(0),
        Angle.fromDegrees(60)
    ),
    SPOT1(
            1,
        Angle.fromDegrees(120),
        Angle.fromDegrees(180)
    ),
    SPOT2(
            2,
        Angle.fromDegrees(240),
        Angle.fromDegrees(300)
    );
    final double NUM_SPOTS = 3;
    Angle intakeAngle;
    Angle outtakeAngle;
    int index;
    SpindexerSpot(int index, Angle intakeAngle, Angle outtakeAngle) {
        this.index = index;
        this.intakeAngle = intakeAngle;
        this.outtakeAngle = outtakeAngle;
    }

    public static SpindexerSpot fromIndex(int index){
        for(SpindexerSpot spot : SpindexerSpot.values()){
            if(spot.index == index){
                return spot;
            }
        }
        return null;
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

}
