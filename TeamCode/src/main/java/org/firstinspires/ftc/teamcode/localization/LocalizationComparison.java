package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.localization.pinpoint.DoublePinpointLocalizer;
import org.firstinspires.ftc.teamcode.old.opModes.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.localization.kalmanFilter.KalmanFilter;
import org.firstinspires.ftc.teamcode.old.opModes.PinpointLocalization;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.DashboardDrawing;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.PoseBuffer;
import org.firstinspires.ftc.teamcode.util.Timer;

@Configurable
@Config
@TeleOp(group = "Localization Comparison", name = "Localization Comparison")
public class LocalizationComparison extends OpMode{
//    public static int currentPipeline = 2;
//    private int previousPipeline = 2;
//    TelemetryManager panelsTelemetry;
    Timer timer;
//    PoseBuffer mt1Buffer;
//    PoseBuffer mt2Buffer;
//    ButtonToggle x1;
//    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
//    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
//
//    IMU imu;
//    AprilTagLocalization aprilTagLocalization;
//    PinpointLocalization pinpointLocalization;


    public final static Pose startingPose = new Pose(0, 0, 0);

//    Pose3D mt1Pose;
//    Pose3D mt2Pose;
//    Position averagedMT1Pose;
//    Position averagedMT2Pose;
//    YawPitchRollAngles imuAngles;
    Pose pinpoint1Pose = new Pose(72, 8, 0);
    Pose pinpoint2Pose = new Pose(72, 8, 0);
    Pose doublePinpointPose = new Pose(72, 8, 0);
    public static double currSpeed = 1.0;

//    Pose2D fusedKalmanPose = new Pose2D(DistanceUnit.INCH, 8, 72, AngleUnit.RADIANS, Math.toRadians(0));
//
//    KalmanFilter kalmanFilter;
//    public static double Q = 0.01;
//    public static double R = 2;
    FtcDashboard dashboard;
//    Follower followerDoublePinpoint;
    Follower followerPinpoint1;
    Follower followerPinpoint2;

    @Override
    public void init() {
        timer = new Timer();

        followerPinpoint1 = ConstantsBot.createPinpointFollower1(hardwareMap);
        followerPinpoint2 = ConstantsBot.createPinpointFollower2(hardwareMap);
//        followerDoublePinpoint = ConstantsBot.createDoublePinpointFollower(hardwareMap);


        followerPinpoint1.setPose(startingPose);
        followerPinpoint2.setPose(startingPose);
//        followerDoublePinpoint.setPose(startingPose);
        followerPinpoint1.startTeleopDrive();


//        mergeDashboardTelemetry();
    }



    private void mergeDashboardTelemetry(){
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    public static double wrap0to2PI(double angle) {
        while (angle < 0) angle += 2*Math.PI;
        while (angle > 2 * Math.PI) angle -= 2*Math.PI;
        return angle;
    }


    @Override
    public void loop() {
        pinpoint1Pose = followerPinpoint1.getPose();
        pinpoint2Pose = followerPinpoint2.getPose();
//
        doublePinpointPose = new Pose((pinpoint1Pose.getX() + pinpoint2Pose.getX()) / 2,
                (pinpoint1Pose.getY() + pinpoint2Pose.getY()) / 2,
                wrap0to2PI((pinpoint1Pose.getHeading() + pinpoint2Pose.getHeading())/2)
        );

        followerPinpoint1.update();
        followerPinpoint2.update();

        followerPinpoint1.setTeleOpDrive(gamepad1.left_stick_y * currSpeed, gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, true);


//        pinpointPose = pinpointLocalization.getCurrentPose();
//        mt1Pose = aprilTagLocalization.getMegaTag1Pose();
//        mt2Pose = aprilTagLocalization.getMegaTag2Pose();
//        averagedMT1Pose = mt1Buffer.update(mt1Pose);
//        averagedMT2Pose = mt2Buffer.update(mt2Pose);


//        if(mt1Pose != null) {
//            double fusedX = kalmanFilter.update(pinpointPose.getX(DistanceUnit.INCH), averagedMT1Pose.x, aprilTagLocalization.aprilTagDetected());
//            double fusedY = kalmanFilter.update(pinpointPose.getY(DistanceUnit.INCH), averagedMT1Pose.y, aprilTagLocalization.aprilTagDetected());
//            double fusedTheta = kalmanFilter.updateAngle(pinpointPose.getHeading(AngleUnit.RADIANS), mt1Pose.getOrientation().getYaw(AngleUnit.RADIANS), aprilTagLocalization.aprilTagDetected());
//            fusedKalmanPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.RADIANS, fusedTheta);
//        }
//        else{
//            double fusedX = kalmanFilter.update(pinpointPose.getX(DistanceUnit.INCH),0, aprilTagLocalization.aprilTagDetected());
//            double fusedY = kalmanFilter.update(pinpointPose.getY(DistanceUnit.INCH), 0, aprilTagLocalization.aprilTagDetected());
//            double fusedTheta = kalmanFilter.updateAngle(pinpointPose.getHeading(AngleUnit.RADIANS), 0, aprilTagLocalization.aprilTagDetected());
//            fusedKalmanPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.RADIANS, fusedTheta);
//        }


        updateTelemetry();
//        updatePanelsTelemetry();

      //  drawPosesDashboard();
//        previousPipeline = currentPipeline;
    }

    private void drawIndividualPose(Pose3D pose, String color){
        DashboardDrawing.drawRobot(new Pose(pose.getPosition().x, pose.getPosition().y, pose.getOrientation().getYaw(AngleUnit.RADIANS)), color);
        PanelsDrawing.drawRobot(new Pose(72 - pose.getPosition().x, pose.getPosition().y - 72, pose.getOrientation().getYaw(AngleUnit.RADIANS)), new Style("", color, 1.0));
    }
    private void drawIndividualPose(Pose2D pose, String color){
        DashboardDrawing.drawRobot(new Pose(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS)), color);
        PanelsDrawing.drawRobot(new Pose(72 - pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH) - 72, pose.getHeading(AngleUnit.RADIANS)), new Style("", color, 1.0));
    }



    private void telemetryPose2D(String s, Pose2D pose){
        telemetry.addData(s + " X", pose.getX(DistanceUnit.INCH));
        telemetry.addData(s + " Y", pose.getY(DistanceUnit.INCH));
        telemetry.addData(s + " Yaw(Deg)", pose.getHeading(AngleUnit.DEGREES));
//        panelsTelemetry.addData(s + " X", pose.getX(DistanceUnit.INCH));
//        panelsTelemetry.addData(s + " Y", pose.getY(DistanceUnit.INCH));
//        panelsTelemetry.addData(s + " Yaw(Deg)", pose.getHeading(AngleUnit.DEGREES));
    }

    private void telemetryPose3D(String s, Pose3D pose){
        telemetry.addData(s + " X", pose.getPosition().x);
        telemetry.addData(s + " Y", pose.getPosition().y);
        telemetry.addData(s + " Yaw(Deg)", pose.getOrientation().getYaw(AngleUnit.DEGREES));
//        panelsTelemetry.addData(s + " X", pose.getPosition().x);
//        panelsTelemetry.addData(s + " Y", pose.getPosition().y);
//        panelsTelemetry.addData(s + " Yaw(Deg)", pose.getOrientation().getYaw(AngleUnit.DEGREES));
    }

    private void updateTelemetry(){
        telemetry.addLine("Update rate" +  1000.0 / timer.getDeltaTime());
        telemetry.addData("Pinpont1 Pose", pinpoint1Pose);
        telemetry.addData("Pinpont2 Pose", pinpoint2Pose);
        telemetry.addData("PinpontDouble Pose", doublePinpointPose);


//        telemetry.addLine("IMU Yaw(Deg): " +  imuAngles.getYaw(AngleUnit.DEGREES));
//        telemetry.addLine("Pinpoint Yaw(Deg)"  + pinpointPose.getHeading(AngleUnit.DEGREES));
//        telemetry.addLine("Mt1 = red, Mt2 = yellow, pinpoint = black, kalman = gray");
//        telemetryPose2D("Pinpoint Pose", pinpointPose);

//        if(mt1Pose != null) {
//            telemetryPose3D("MT1 Pose", mt1Pose);
//            telemetryPose3D("MT1 Pose Avged", new Pose3D(averagedMT1Pose, mt1Pose.getOrientation()));
//            telemetryPose2D("Fused Kalman Pose", fusedKalmanPose);
//        }
//
//        if(mt2Pose!= null) {
//            telemetryPose3D("MT2 Pose", mt2Pose);
//            telemetryPose3D("MT2 Pose Avged", new Pose3D(averagedMT2Pose, mt2Pose.getOrientation()));
//        }
        telemetry.update();
    }




}
