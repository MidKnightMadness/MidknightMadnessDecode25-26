package org.firstinspires.ftc.teamcode.localization.AprilTag;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Util.ConfigNames;
import org.firstinspires.ftc.teamcode.Util.PoseBuffer;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Deprecated

public class AprilTagLocalization {

    Limelight3A limelight;

    String configName = ConfigNames.externalUSB;
    int leftPipeline = 0;
    int rightPipeline = 2;

    int currentPipeline = rightPipeline;

    int previousPipelineNum = leftPipeline;

    double cameraToCenterXOffset = 0;//0.2032 * 1000 / 25.4;
    double cameraToCenterYOffset = 0;//0.217 * 1000 /25.4;
    Timer timer;
    PoseBuffer mt1Buffer;
    PoseBuffer mt2Buffer;
    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;
    Pose2D initPose;
    Pose3D megaTag1Pose;
    Pose3D megaTag2Pose;


    //Pose (0,0) is on the back left side
    PoseBuffer tag1Buffer;
    PoseBuffer tag2Buffer;

    IMU imu;
    boolean detected;


    public AprilTagLocalization(HardwareMap hardwareMap, Pose2D initPos){
        timer = new Timer();
        limelight = hardwareMap.get(Limelight3A.class, configName);
        limelight.pipelineSwitch(currentPipeline);//set it to motif pipeline

        limelight.start();

        mt1Buffer = new PoseBuffer();
        mt2Buffer = new PoseBuffer();

        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)));

        this.initPose = initPos;
        tag1Buffer = new PoseBuffer();
        tag2Buffer = new PoseBuffer();
    }

    public void swapPipeline(int pipeline){
        if(pipeline == leftPipeline || pipeline == rightPipeline){
            limelight.pipelineSwitch(pipeline);
        }
    }

    public boolean aprilTagDetected(){
        return detected;
    }
    public Pose3D getMegaTag1Pose(){
        return megaTag1Pose;
    }


    public Pose3D getMegaTag2Pose(){
        return megaTag2Pose;
    }

    public void update(){
        YawPitchRollAngles imuAngles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(imuAngles.getYaw() + Math.toRadians(180));


        timer.updateTime();

        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            double time = result.getStaleness();
            detected = true;
            megaTag1Pose = result.getBotpose();
            megaTag1Pose = offsetToBackLeftOrigin(convertMetersToInch(megaTag1Pose), 72, 72);

            megaTag2Pose = result.getBotpose_MT2();
            megaTag2Pose = offsetToBackLeftOrigin(convertMetersToInch(megaTag2Pose), 72, 72);
        }

        else{
            detected = false;
        }

    }

    private Pose3D offsetToBackLeftOrigin(Pose3D pose3D, double xOffset, double yOffset) {
        Position position = pose3D.getPosition();
        double x = -position.x + xOffset - cameraToCenterXOffset;//reverse so positive = forward
        double y = position.y + yOffset - cameraToCenterYOffset;
        YawPitchRollAngles yawPitchRollAngles = pose3D.getOrientation();
        return new Pose3D(new Position(DistanceUnit.INCH, x, y, position.z, 0 ), new YawPitchRollAngles(AngleUnit.RADIANS, yawPitchRollAngles.getYaw(AngleUnit.RADIANS) + Math.PI * 2, yawPitchRollAngles.getPitch(), yawPitchRollAngles.getRoll(), 0));
    }


    private Pose3D convertMetersToInch(Pose3D pose){
        Position position = pose.getPosition();
        double x = position.x * 100 / 2.54;
        double y = position.y * 100 / 2.54;
        double z = position.z * 100 / 2.54;
        return new Pose3D(new Position(DistanceUnit.INCH, x, y, z, 0), pose.getOrientation());

    }






}
