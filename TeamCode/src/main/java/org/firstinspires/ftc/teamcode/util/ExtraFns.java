package org.firstinspires.ftc.teamcode.util;

import static com.pedropathing.math.MathFunctions.normalizeAngle;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Map;

public class ExtraFns {
    public static Angle getAngle(Pose position, Pose target) {
        Pose gap = target.minus(position);
        double heading = Math.atan2(gap.getX(), gap.getY());
        return new Angle(heading + position.getHeading(), AngleUnit.RADIANS);
    }

    public static double getAngleError(Pose position, Pose target){
        double deltaY = target.getY() - position.getY();
        double deltaX = target.getX() - position.getX();
        double heading = Math.atan2(deltaY, deltaX);

        boolean deltaXPositive = (deltaX > 0);
        boolean deltaYPositive = (deltaY > 0);

        if((!deltaXPositive && deltaYPositive) || (!deltaXPositive && !deltaYPositive)){
            heading += Math.PI;
        }
        heading = normAngle(heading);
        //heading is in absolute degrees
        double error = heading - position.getHeading();
        double errorSign = (error > 0 ) ? -1 : 1;
        if(Math.abs(error) > Math.PI){
            error =  errorSign * (2 * Math.PI - Math.abs(position.getHeading() - heading));
        }

        return normAngle(error);
    }


    public static double normAngle(double val){
        while(val < 0){
            val += Math.PI * 2;
        }
        while(val > 2 * Math.PI){
            val -= Math.PI * 2;
        }
        return val;
    }
    public static <K, V extends Comparable<? super V>> K argmax(Map<K, V> map) {
        return map.entrySet()
                .stream()
                .max(Map.Entry.comparingByValue())
                .map(Map.Entry::getKey)
                .orElse(null); // or throw if you prefer
    }
}
