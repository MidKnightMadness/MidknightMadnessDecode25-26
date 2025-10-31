package org.firstinspires.ftc.teamcode.hardware;

import com.seattlesolvers.solverslib.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public interface Encoder extends HardwareDevice {
    /**
     * @return The normalized angular position of the encoder
     */
    double getAngle();

    /**
     * @return The angle unit associated with the encoder
     */
    AngleUnit getAngleUnit();

    /**
     * @return Direction multiplier: 1 for normal, -1 for reversed
     */
    int getDirectionMultiplier();

    /**
     * Sets whether or not the encoder should be reversed for any future values returned when reading the encoder
     * @param direction The direction of the encoder should be reversed for any future values
     * @return The object itself for chaining purposes
     */
    Encoder setDirection(RotationDirection direction);

    /**
     * Sets whether or not the encoder should be reversed for any future values returned when reading the encoder
     * @param reversed Whether or not the encoder should be reversed for any future values
     * @return The object itself for chaining purposes
     */
    Encoder setReversed(boolean reversed);

    /**
     * @return The direction of the encoder
     */
    RotationDirection getDirection();

    /**
     * @return Whether or not an encoder is reversed
     */
    boolean getReversed();

    /**
     * Resets the encoder for any future values returned
     * @return The object itself for chaining purposes
     */
    Encoder zero();

    /**
     * Resets the encoder to a particular angle
     * @return The object itself for chaining purposes
     */
    Encoder setAngle(double angle);

    /**
     * Resets the offset for any future values returned
     */
    void resetOffset();
}
