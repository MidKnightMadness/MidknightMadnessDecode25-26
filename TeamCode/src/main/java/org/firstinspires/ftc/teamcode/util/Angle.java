package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Angle {
    private final double value;
    private final AngleUnit unit;

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
        return (unit == AngleUnit.DEGREES) ? this.toDegrees() : this.toRadians();
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

    // Arithmetic operations
    public Angle add(Angle other) {
        double sum = this.toRadians() + other.toRadians();
        return new Angle(sum, unit).wrap();
    }

    public Angle sub(Angle other) {
        double diff = this.toRadians() - other.toRadians();
        return new Angle(diff, unit).wrap();
    }

    public Angle absGap(Angle other) {
        return this.sub(other).abs();
    }

    public Angle abs() {
        return new Angle(Math.abs(value), unit);
    }

    public Angle neg() {
        return new Angle(-value, unit);
    }

    @NonNull
    @Override
    public String toString() {
        return value + " " + unit;
    }
}