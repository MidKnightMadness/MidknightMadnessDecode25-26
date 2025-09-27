package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Configurable
@Autonomous(group = "Testing")
public class LinearPath extends OpMode {
    private Follower follower;
    private PathChain pathChain;

    private Timer timer;
    private TelemetryManager telemetryM;

    public static Pose startPose = new Pose(0, 72, Math.toRadians(0));
    public static Pose endPose = new Pose(72, 72, Math.toRadians(0));

    @Override
    public void init() {
        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        timer.restartTimer();
    }

    @Override
    public void loop() {
        follower.update();
        follower.followPath(pathChain, true);

        telemetryM.addData("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.addData("Drive PIDF", follower.constants.coefficientsDrivePIDF);
        telemetryM.addData("Is busy", follower.isBusy());
        telemetryM.update(telemetry);
    }
}
