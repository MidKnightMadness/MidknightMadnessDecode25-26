package org.firstinspires.ftc.teamcode.Motors;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp
public class AccelTest extends OpMode {
    public static double accelSpeed = 1;
    public static double decelSpeed = -1;

    TelemetryManager telemetryM;
    Follower follower;
    WheelControl wheelControl;
    Timer timer;
    Pose startPose;

    Pose pose;
    double distanceFromStart;
    double speed;
    double acceleration;

    ArrayList<PoseData> accelHistory;
    ArrayList<PoseData> decelHistory;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        wheelControl = new WheelControl(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        timer = new Timer();
        timer.restartTimer();
        startPose = follower.getPose();
        accelHistory = new ArrayList<>();
        decelHistory = new ArrayList<>();
    }

    @Override
    public void loop() {
        pose = follower.getPose();
        distanceFromStart = pose.distanceFrom(startPose);
        speed = follower.getVelocity().getMagnitude();
        acceleration = follower.getAcceleration().getMagnitude();
        PoseData poseData = new PoseData(pose, speed, acceleration);

        if (distanceFromStart < 50) {
            wheelControl.drive_relative(accelSpeed, 0, 0, 1);
            accelHistory.add(poseData);
        } else {
            wheelControl.drive_relative(decelSpeed, 0, 0, 1);
            decelHistory.add(poseData);
        }
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetryM.addData("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.addData("Pose X", pose.getX());
        telemetryM.addData("Pose Y", pose.getY());
        telemetryM.addData("Pose Heading", pose.getHeading());
        telemetryM.addData("Distance form start", distanceFromStart);
        telemetryM.addData("Speed", speed);
        telemetryM.addData("Acceleration", acceleration);
        telemetryM.update(telemetry);
    }
}

class PoseData {
    Pose pose;
    double speed;
    double acceleration;

    public PoseData(Pose pose, double speed, double acceleration) {
        this.pose = pose;
        this.speed = speed;
        this.acceleration = acceleration;
    }
}
