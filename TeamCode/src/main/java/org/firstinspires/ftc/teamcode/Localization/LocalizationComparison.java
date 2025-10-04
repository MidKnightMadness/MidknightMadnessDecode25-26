package org.firstinspires.ftc.teamcode.Localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Localization.AprilTag.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.Localization.Pinpoint.PinpointLocalization;
import org.firstinspires.ftc.teamcode.Util.ButtonToggle;
import org.firstinspires.ftc.teamcode.Util.PoseBuffer;
import org.firstinspires.ftc.teamcode.Util.Timer;

@TeleOp(group = "Localization", name = "LocalizationComparison")
@Configurable
public class LocalizationComparison extends OpMode{
    public static int currentPipeline = 2;
    private int previousPipeline = 2;
    TelemetryManager panelsTelemetry;
    Timer timer;
    PoseBuffer mt1Buffer;
    PoseBuffer mt2Buffer;
    ButtonToggle x1;
    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
    Pose3D megaTag1Pose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS.ordinal(), 0), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));
    Pose3D megaTag2Pose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS.ordinal(), 0), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));

    IMU imu;
    AprilTagLocalization aprilTagLocalization;
    PinpointLocalization pinpointLocalization;


    static Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 8, 72, AngleUnit.RADIANS, Math.toRadians(90));

    Pose3D mt1Pose = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));

    Pose3D mt2Pose = new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0));

    Position averagedMT1Pose = new Position();
    Position averagedMT2Pose = new Position();
    YawPitchRollAngles imuAngles;
    Pose2D pinpointPose = new Pose2D(DistanceUnit.INCH, 8, 72, AngleUnit.RADIANS, Math.toRadians(90));

    @Override
    public void init() {
        timer = new Timer();
        telemetry.setMsTransmissionInterval(10);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        pinpointLocalization = new PinpointLocalization(hardwareMap, startingPose);
        aprilTagLocalization = new AprilTagLocalization(hardwareMap, startingPose);


        x1 = new ButtonToggle();
        mt1Buffer = new PoseBuffer();
        mt2Buffer = new PoseBuffer();

        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)));
    }



    private void updateDashboardTelemetry(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }



    @Override
    public void loop() {
        if(previousPipeline != currentPipeline){
            aprilTagLocalization.swapPipeline(currentPipeline);
            telemetry.addLine("Updating Pipeline to " + currentPipeline);
        }

        imuAngles = imu.getRobotYawPitchRollAngles();

        aprilTagLocalization.update();
        pinpointLocalization.update();


        mt1Pose = aprilTagLocalization.getMegaTag1Pose();
        mt2Pose = aprilTagLocalization.getMegaTag2Pose();
        averagedMT1Pose = mt1Buffer.update(mt1Pose);
        averagedMT2Pose = mt2Buffer.update(mt2Pose);

        updateDashboardTelemetry();

        timer.updateTime();

        updateTelemetry();
        updatePanelsTelemetry();
    }

    private void updateTelemetry(){
        telemetry.addData("Update rate", 1/ timer.getDeltaTime());
        telemetry.addData("Yaw(Deg): ", imuAngles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("MT1Pose", mt1Pose.getPosition() != null ? mt1Pose.getPosition() : "Null");
        telemetry.addData("MT1Position Averaged", averagedMT1Pose != null ? averagedMT1Pose: "Null");
        telemetry.addData("MT2Pose", mt2Pose.getPosition() != null ? mt2Pose.getPosition() : "Null");
        telemetry.addData("MT2Position Averaged", averagedMT2Pose != null ? averagedMT2Pose : "Null");
        telemetry.update();
    }


    private void updatePanelsTelemetry(){
        panelsTelemetry.addData("Update rate", 1/ timer.getDeltaTime()) ;
        panelsTelemetry.addData("Yaw(Deg): ", imuAngles.getYaw(AngleUnit.DEGREES));
        panelsTelemetry.addData("MT1Pose", mt1Pose.getPosition() != null ? mt1Pose.getPosition() : "Null");
        panelsTelemetry.addData("MT1Position Averaged", averagedMT1Pose != null ? averagedMT1Pose: "Null");
        panelsTelemetry.addData("MT2Pose", mt2Pose.getPosition() != null ? mt2Pose.getPosition() : "Null");
        panelsTelemetry.addData("MT2Position Averaged", averagedMT2Pose != null ? averagedMT2Pose : "Null");
        panelsTelemetry.update();
    }

}
