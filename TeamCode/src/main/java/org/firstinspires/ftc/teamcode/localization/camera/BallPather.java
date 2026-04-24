package org.firstinspires.ftc.teamcode.localization.camera;

import com.pedropathing.geometry.Pose;

public interface BallPather {
    Pose[] findPath(Pose robotPose, Pose[] ballPoses, int nBalls);
}
