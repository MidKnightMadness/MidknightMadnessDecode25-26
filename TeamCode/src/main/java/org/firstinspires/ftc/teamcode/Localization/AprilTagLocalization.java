package org.firstinspires.ftc.teamcode.Localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Util.ButtonToggle;
import org.firstinspires.ftc.teamcode.Util.ConfigNames;
import org.firstinspires.ftc.teamcode.Util.PoseBuffer;
import org.firstinspires.ftc.teamcode.Util.Timer;

import java.util.List;


public class AprilTagLocalization {

    Limelight3A limelight;

    String configName = ConfigNames.externalUSB;
    int leftPipeline = 0;
    int rightPipeline = 2;

    int currentPipeline = leftPipeline;

    int previousPipelineNum = leftPipeline;
    Timer timer;
    PoseBuffer mt1Buffer;
    PoseBuffer mt2Buffer;
    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
    Pose2D initPose;
    Pose2D megaTag1Pose;
    Pose2D megaTag2Pose;


    //Pose (0,0) is on the back left side
    PoseBuffer tag1Buffer;
    PoseBuffer tag2Buffer;

    IMU imu;

    public void initialize(HardwareMap hardwareMap, Pose2D initPos){
        timer = new Timer();
        limelight = hardwareMap.get(Limelight3A.class, configName);
        limelight.pipelineSwitch(currentPipeline);//set it to motif pipeline

        limelight.start();

        mt1Buffer = new PoseBuffer();
        mt2Buffer = new PoseBuffer();

        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)));

        this.initPose = initPos;
        tag1Buffer = new PoseBuffer();
        tag2Buffer = new PoseBuffer();
    }


    public void update(){
        YawPitchRollAngles imuAngles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(imuAngles.getYaw(AngleUnit.DEGREES));


        timer.updateTime();

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D megaTag1Pose = result.getBotpose();
            megaTag1Pose = offsetToBackLeftOrigin(convertMetersToInch(megaTag1Pose));

            Pose3D megaTag2Pose = result.getBotpose_MT2();
            megaTag2Pose = convertMetersToInch(megaTag2Pose);

        }

    }

    private Pose3D offsetCenter(Pose3D pose3D, double xOffset, double yOffset) {
        Position position = pose3D.getPosition();
        double x = position.x + xOffset;
        double y = position.y + yOffset;
        return new Pose3D(new Position(x, y))
    }

    public Pose3D convertMetersToInch(Pose3D pose){
        Position position = pose.getPosition();
        double x = position.x * 100 / 2.54;
        double y = position.y * 100 / 2.54;
        double z = position.z * 100 / 2.54;
        return new Pose3D(new Position(DistanceUnit.INCH, x, y, z, 0), pose.getOrientation());

    }


    public void loop() {


        if(previousPipelineNum != currentPipeline){
            limelight.pipelineSwitch(currentPipeline);
        }




            previousPipelineNum = currentPipeline;


    }



}
