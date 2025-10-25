package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

import java.lang.annotation.Annotation;
import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "DriveTeleOp")
public class DriveTeleOp extends OpMode {
    private Follower follower;
    public static Pose startPose = new Pose(72, 203.2 /25.4, Math.toRadians(90));

    TelemetryManager telemetryM;
    GraphManager graphM;

    @Override
    public void init() {
        Drawing.init();
        follower = Constants.createKalmanPinpointAprilFollower(hardwareMap, startPose);
        follower.setStartingPose(startPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();

    }
    public static double maxSpeed = 0.8;
    public static double midSpeed = 0.5;
    public static double currSpeed = 0.8;

    @Override
    public void start(){
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_x * currSpeed, -gamepad1.left_stick_y * currSpeed, -gamepad1.right_stick_x * currSpeed, true);


        if(gamepad1.yWasPressed()){
             currSpeed = maxSpeed;
        }
        if (gamepad1.bWasPressed()) {
            currSpeed = midSpeed;
        }


        telemetry.addData("Robot Pos", follower.getPose());
        telemetry.addData("Speed", currSpeed);
        updatePanelsTelemetry();

    }

    public void updatePanelsTelemetry() {
        // Field
        Drawing.drawRobot(follower.getPose());
        Drawing.drawPoseHistory(follower.getPoseHistory());
        Drawing.sendPacket();
//
//        // Telemetry
//        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
//        telemetryM.addData("Pose X (in)", currentPose.getX());
//        telemetryM.addData("Pose Y (in)", currentPose.getY());
//        telemetryM.addData("Pose Heading (rad)", currentPose.getHeading());
//        addDataTelemetryGraph("Speed (in/s)", speed);
//        addDataTelemetryGraph("Acceleration (in/s^2)", acceleration);

        // Updates
        telemetryM.update(telemetry);
        graphM.update();
    }
}
