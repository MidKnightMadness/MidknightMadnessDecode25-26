package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class EncoderBase<T extends EncoderBase<T>> implements Encoder {
    protected double offset = 0.0;
    protected RotationDirection direction = RotationDirection.FORWARD;
    protected AngleUnit angleUnit;

    @Override
    public int getDirectionMultiplier() {
        return direction.getMultiplier();
    }

    @Override
    @SuppressWarnings("unchecked")
    public T setDirection(RotationDirection direction) {
        this.direction = direction;
        return (T) this;
    }

    @Override
    @SuppressWarnings("unchecked")
    public T setReversed(boolean reversed) {
        direction = reversed ? RotationDirection.REVERSE : RotationDirection.FORWARD;
        return (T) this;
    }

    @Override
    public RotationDirection getDirection() {
        return direction;
    }

    @Override
    public boolean getReversed() {
        return direction == RotationDirection.REVERSE;
    }

    @Override
    public AngleUnit getAngleUnit() {
        return angleUnit;
    }

    @Override
    public void resetOffset() {
        this.offset = 0;
    }
}