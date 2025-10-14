package org.firstinspires.ftc.teamcode.motors;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp
public class AccelTest extends OpMode {
    public static double accelSpeed = 1;
    public static double decelSpeed = -1;

    private static final Style accelHistoryLook = new Style(
            "", "#4CAF50", 0.75
    );
    private static final Style decelHistoryLook = new Style(
            "", "#C22915", 0.75
    );

    TelemetryManager telemetryM;
    GraphManager graphM;
    Follower follower;
    WheelControl wheelControl;
    Timer timer;

    Pose startPose = new Pose(72, 30, Math.toRadians(90));
    Pose pose;
    double distanceFromStart;
    double speed;
    double acceleration;

    enum State {
        accel,
        decel,
        stop
    }
    State state;

    ArrayList<Pose> accelHistory;
    ArrayList<Pose> decelHistory;

    @Override
    public void init() {
        Drawing.init();
        timer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        wheelControl = new WheelControl(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        accelHistory = new ArrayList<>();
        decelHistory = new ArrayList<>();
        state = State.accel;
    }

    @Override
    public void init_loop() {
        updateData();
        updateTelemetry();
    }

    @Override
    public void start() {
        timer.restartTimer();
    }

    @Override
    public void loop() {
        updateData();
        switch (state) {
            case accel:
                wheelControl.drive_relative(accelSpeed, 0, 0, 1);
                accelHistory.add(pose);
                if (distanceFromStart > 70) {
                    state = State.decel;
                }
                break;

            case decel:
                wheelControl.drive_relative(decelSpeed, 0, 0, 1);
                decelHistory.add(pose);
                if (distanceFromStart < 20) {
                    state = State.stop;
                }
                break;

            case stop:
                wheelControl.stop();
                break;
        }
        updateTelemetry();
    }

    public void updateData() {
        follower.update();
        pose = follower.getPose();
        distanceFromStart = pose.distanceFrom(startPose);
        speed = follower.getVelocity().getMagnitude();
        acceleration = follower.getAcceleration().getMagnitude();
    }

    public void addDataTeleGraph(String key, Number value) {
        telemetryM.addData(key, value);
        graphM.addData(key, value);
    }

    public void updateTelemetry() {
        // Field plugin
        Drawing.drawRobot(pose);
        Drawing.drawPoseHistoryAL(accelHistory, accelHistoryLook);
        Drawing.drawPoseHistoryAL(decelHistory, decelHistoryLook);
        Drawing.sendPacket();

        // Telemetry
        addDataTeleGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.addData("Pose X (in)", pose.getX());
        telemetryM.addData("Pose Y (in)", pose.getY());
        telemetryM.addData("Pose Heading (rad)", pose.getHeading());
        telemetryM.addData("State", state);
        addDataTeleGraph("Distance from start (in)", distanceFromStart);
        addDataTeleGraph("Speed (in/s)", speed);
        addDataTeleGraph("Acceleration (in/s^2)", acceleration);

        telemetryM.update(telemetry);
        graphM.update();
    }
}