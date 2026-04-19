package org.firstinspires.ftc.teamcode.util;
import androidx.annotation.NonNull;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;

public class AngleNonCR {
    private double value;
    private AngleUnit unit;

    public AngleNonCR(double value, AngleUnit unit) {
        this.value = value;
        this.unit = unit;
    }

    // Factory methods
    public static AngleNonCR fromDegrees(double deg) {
        return new AngleNonCR(deg, AngleUnit.DEGREES);
    }

    public static AngleNonCR fromRadians(double rad) {
        return new AngleNonCR(rad, AngleUnit.RADIANS);
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

    public AngleNonCR changeUnit(AngleUnit angleUnit) {
        value = this.to(angleUnit);
        unit = angleUnit;
        return this;
    }

    public AngleNonCR wrap() {
        double wrapped;
        switch (unit) {
            case RADIANS:
                wrapped = value % (SpindexerNonCR.totalDegrees * Math.PI / 180);
                break;
            case DEGREES:
                wrapped = value % 480;
                break;
            default: throw new IllegalStateException("Unknown unit");
        }
        return new AngleNonCR(wrapped, unit);
    }

    // Arithmetic operations
    public AngleNonCR add(Angle other) {
        double sum = this.to(unit) + other.to(unit);
        return new AngleNonCR(sum, unit).wrap();
    }

    public AngleNonCR sub(Angle other) {
        double diff = this.to(unit) - other.to(unit);
        return new AngleNonCR(diff, unit).wrap();
    }

    public AngleNonCR absGap(Angle other) {
        return this.sub(other).abs();
    }

    public AngleNonCR diff(AngleNonCR b) {
        if(b.toDegrees() >= this.toDegrees()){
            return AngleNonCR.fromDegrees(b.toDegrees() - this.toDegrees());
        } else{
            return AngleNonCR.fromDegrees(this.toDegrees() - b.toDegrees());
        }
    }
    public AngleNonCR abs() {
        return new AngleNonCR(java.lang.Math.abs(value), unit);
    }

    public AngleNonCR neg() {
        return new AngleNonCR(-value, unit);
    }

    public int sign() {
        return (int) Math.signum(value);
    }

    @NonNull
    @Override
    public String toString() {
        return value + " " + unit;
    }
}