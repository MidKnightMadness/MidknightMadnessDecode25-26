package org.firstinspires.ftc.teamcode.pedroPathing.robotDrive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;

@Configurable
@TeleOp(name = "DriveNoFollower")
public class DriveNoLocalizer extends OpMode {
    WheelControl wheelControl;
    TelemetryManager telemetryM;
    GraphManager graphM;

    @Override
    public void init() {
        PanelsDrawing.init();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        wheelControl = new WheelControl(hardwareMap);
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        graphM = PanelsGraph.INSTANCE.getManager();

    }
//    public static double maxSpeed = 0.8;
//    public static double midSpeed = 0.5;
    public static double currSpeed = 0.8;

    public static double maxPower = 0.8;
    @Override
    public void start(){
    }
    @Override
    public void loop() {
        wheelControl.drive_relative(-gamepad1.left_stick_y * currSpeed, gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, 0.8);
//        follower.setTeleOpDrive(-gamepad1.left_stick_y * currSpeed, -gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, true);
//
//
//        if(gamepad1.yWasPressed()){
//            currSpeed = maxSpeed;
//        }
//        if (gamepad1.bWasPressed()) {
//            currSpeed = midSpeed;
//        }


//        telemetry.addData("Robot PosX:", follower.getPose());
//        telemetry.addData("Speed", currSpeed);
//        updatePanelsTelemetry();
//        PanelsDrawing.drawRobot(follower.getPose());
//        PanelsDrawing.sendPacket();
    }

    public void updatePanelsTelemetry() {
        // Field
//        PanelsDrawing.drawRobot(follower.getPose());
//        PanelsDrawing.drawPoseHistory(follower.getPoseHistory());
//        PanelsDrawing.sendPacket();
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
