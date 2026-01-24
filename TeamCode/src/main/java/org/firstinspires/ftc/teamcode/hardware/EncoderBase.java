package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Angle;

public abstract class EncoderBase<T extends EncoderBase<T>> implements Encoder {
    protected double offset = 0.0;
    protected RotationDirection direction = RotationDirection.FORWARD;
    protected AngleUnit angleUnit;

    @Override
    public int getDirectionMultiplier() {
        return direction.getMultiplier();
    }

    //sets the direction encoder is going
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
    //sets encoder value to 0
    @Override
    @SuppressWarnings("unchecked")
    public T zero() {
        this.setAngle(0);
        return (T) this;
    }
    //gets direction
    @Override
    public RotationDirection getDirection() {
        return direction;
    }

    @Override
    public boolean getReversed() {
        return direction == RotationDirection.REVERSE;
    }//gets whether its reversed

    @Override
    public AngleUnit getAngleUnit() {
        return angleUnit;
    }//gets the angle

    @Override
    public void resetOffset() {
        this.offset = 0;
    }//resets the offset
}