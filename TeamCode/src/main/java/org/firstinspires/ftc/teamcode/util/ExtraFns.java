package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Map;

public class ExtraFns {
    public static Angle getAngle(Pose position, Pose target) {
        Pose gap = target.minus(position);
        return new Angle(Math.atan2(gap.getY(), gap.getX()), AngleUnit.RADIANS);
    }

    public static <K, V extends Comparable<? super V>> K argmax(Map<K, V> map) {
        return map.entrySet()
                .stream()
                .max(Map.Entry.comparingByValue())
                .map(Map.Entry::getKey)
                .orElse(null); // or throw if you prefer
    }
}
