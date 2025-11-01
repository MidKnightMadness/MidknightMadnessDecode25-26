package org.firstinspires.ftc.teamcode.pedroPathing.robotDrive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;

@Configurable
@TeleOp(name = "DriveTeleOp")
public class DriveTeleOp extends OpMode {
    private Follower follower;
    public static Pose startPose = new Pose(72, 8, Math.toRadians(90));

    TelemetryManager telemetryM;
    GraphManager graphM;

    @Override
    public void init() {
        ConstantsBot.motifIsBusy = false;
        PanelsDrawing.init();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
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

        follower.setTeleOpDrive(-gamepad1.left_stick_y * currSpeed, -gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, true);


        if(gamepad1.yWasPressed()){
             currSpeed = maxSpeed;
        }
        if (gamepad1.bWasPressed()) {
            currSpeed = midSpeed;
        }


//        telemetry.addData("Robot PosX:", follower.getPose());
//        telemetry.addData("Speed", currSpeed);
//        updatePanelsTelemetry();
        PanelsDrawing.drawRobot(follower.getPose());
        PanelsDrawing.sendPacket();
    }

    public void updatePanelsTelemetry() {
        // Field
        PanelsDrawing.drawRobot(follower.getPose());
//        PanelsDrawing.drawPoseHistory(follower.getPoseHistory());
        PanelsDrawing.sendPacket();
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
