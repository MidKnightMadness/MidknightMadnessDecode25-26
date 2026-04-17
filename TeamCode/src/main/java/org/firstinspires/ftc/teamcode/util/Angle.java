package org.firstinspires.ftc.teamcode.util;
import androidx.annotation.NonNull;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Angle {
    private double value;
    private AngleUnit unit;

    public Angle() {
        this(0, AngleUnit.RADIANS);
    }

    public Angle(double value, AngleUnit unit) {
        this.value = value;
        this.unit = unit;
    }

    // Factory methods
    public static Angle fromDegrees(double deg) {
        return new Angle(deg, AngleUnit.DEGREES);
    }

    public static Angle fromRadians(double rad) {
        return new Angle(rad, AngleUnit.RADIANS);
    }

    // Getters
    public double getValue() {
        return value;
    }

    public AngleUnit getUnit() {
        return unit;
    }

    // Conversion
    public double toRadians() {
        return (unit == AngleUnit.RADIANS) ? value : Math.toRadians(value);
    }

    public double toDegrees() {
        return (unit == AngleUnit.DEGREES) ? value : Math.toDegrees(value);
    }

    public double to(AngleUnit angleUnit) {
        return (angleUnit == AngleUnit.DEGREES) ? this.toDegrees() : this.toRadians();
    }

    public Angle changeUnit(AngleUnit angleUnit) {
        value = this.to(angleUnit);
        unit = angleUnit;
        return this;
    }

    public Angle wrap() {
        double wrapped;
        switch (unit) {
            case RADIANS:
                wrapped = AngleUnit.normalizeRadians(value);
                break;
            case DEGREES:
                wrapped = AngleUnit.normalizeDegrees(value);
                break;
            default: throw new IllegalStateException("Unknown unit");
        }
        return new Angle(wrapped, unit);
    }

    public Angle wrapUnsigned() {
        double wrapped;
        switch (unit) {
            case RADIANS:
                wrapped = AngleUnit.normalizeRadians(value);
                if (wrapped < 0) wrapped += 2 * Math.PI;
                break;
            case DEGREES:
                wrapped = AngleUnit.normalizeDegrees(value);
                if (wrapped < 0) wrapped += 360;
                break;
            default: throw new IllegalStateException("Unknown unit");
        }
        return new Angle(wrapped, unit);
    }

    // Arithmetic operations
    public Angle add(Angle other) {
        double sum = this.to(unit) + other.to(unit);
        return new Angle(sum, unit);
    }

    public Angle sub(Angle other) {
        double diff = this.to(unit) - other.to(unit);
        return new Angle(diff, unit);
    }

    public Angle delta(Angle other) {
        return this.sub(other).wrap();
    }

    public Angle distance(Angle other) {
        return this.delta(other).abs();
    }

    public Angle abs() {
        return new Angle(java.lang.Math.abs(value), unit);
    }

    public Angle neg() {
        return new Angle(-value, unit);
    }

    public int sign() {
        return (int) Math.signum(value);
    }

    public boolean less(Angle other) {
        return this.toRadians() < other.toRadians();
    }

    public boolean greater(Angle other) {
        return this.toRadians() > other.toRadians();
    }

    public boolean le(Angle other) {
        return this.toRadians() <= other.toRadians();
    }

    public boolean ge(Angle other) {
        return this.toRadians() >= other.toRadians();
    }

    public boolean atAngle(Angle other, Angle tolerance) {
        return this.distance(other).le(tolerance);
    }

    @NonNull
    @Override
    public String toString() {
        return value + " " + unit;
    }
}