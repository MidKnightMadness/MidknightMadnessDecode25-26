package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.geometry.Pose;

public enum ShootSide {
    LEFT,
    RIGHT;

    public Pose fromLeftPose(Pose pose) {
        return this == ShootSide.LEFT ? pose : pose.mirror();
    }

    public double fromLeftHeading(double heading) {
        return this == ShootSide.LEFT ? heading : Math.PI - heading;
    }

    public double fromLeftX(double x) {
        return this == ShootSide.LEFT ? x : 141.5 - x;
    }

    public Pose toLeftPose(Pose pose) {
        return fromLeftPose(pose);
    }

    public double toLeftHeading(double heading) {
        return fromLeftHeading(heading);
    }

    public double toLeftX(double x) {
        return fromLeftX(x);
    }
}
