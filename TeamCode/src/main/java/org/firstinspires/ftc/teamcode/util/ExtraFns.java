package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Map;
import java.lang.Math;
import java.util.function.BooleanSupplier;

public class ExtraFns {
    public static Angle getAngle(Pose position, Pose target) {
        Pose gap = target.minus(position);
        double heading = Math.atan2(gap.getX(), gap.getY());
        return new Angle(heading + position.getHeading(), AngleUnit.RADIANS);
    }

    public static double getTargetAngle(Pose position, Pose target) {
        double deltaY = target.getY() - position.getY();
        double deltaX = target.getX() - position.getX();
        double heading = Math.atan2(deltaY, deltaX);
        return normAngle(heading);
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

    public static double dotPose(Pose a, Pose b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static double normAnglePlusMinusPI(double error) {
        while (error < -Math.PI) {
            error += Math.PI * 2;
        }
        while (error > Math.PI) {
            error -= Math.PI * 2;
        }
        return error;
    }

    public static BooleanSupplier firstSupplier(BooleanSupplier condition) {
        return new BooleanSupplier() {
            private boolean hasRun = false;
            @Override
            public boolean getAsBoolean() {
                if (!hasRun && condition.getAsBoolean()) {
                    hasRun = true;
                    return true;
                }
                return false;
            }
        };
    }

    public static double distToLine(Pose robotPose, Pose c1, Pose c2) {
        double numerator = (c2.getX() - c1.getX()) * (robotPose.getY() - c1.getY())
                - (c2.getY() - c1.getY()) * (robotPose.getX() - c1.getX());
        double denominator = c2.minus(c1).getAsVector().getMagnitude();
        return numerator / denominator;
    }

    public static double closeZoneDist(Pose robotPose) {
        Pose c1 = new Pose(70.75, 70.75), c2;
        if (robotPose.getX() < 70.75) {
            c2 = new Pose(0, 141.5);
            return Math.abs(distToLine(robotPose, c1, c2));
        } else {
            c2 = new Pose(141.5, 141.5);
            return Math.abs(distToLine(robotPose, c1, c2));
        }
    }

    public static double farZoneDist(Pose robotPose) {
        Pose c1 = new Pose(70.75, 24), c2;
        if (robotPose.getX() < 70.75) {
            c2 = new Pose(48, 0);
            return Math.abs(distToLine(robotPose, c1, c2));
        } else {
            c2 = new Pose(96, 0);
            return Math.abs(distToLine(robotPose, c1, c2));
        }
    }
}
