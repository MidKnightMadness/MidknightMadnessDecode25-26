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
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Util.ButtonToggle;
import org.firstinspires.ftc.teamcode.Util.PoseBuffer;
import org.firstinspires.ftc.teamcode.Util.Timer;

import java.util.List;

@TeleOp(group = "Vision", name = "AprilTagTesting")
@Configurable
public class AprilTagLocalization extends OpMode{

    private Limelight3A limelight;
    public static String configName = "limelight";
    private static int leftPipeline = 0;
    private static int motifPipeline = 1;
    private static int rightPipeline = 2;

    public static int currentPipeline = leftPipeline;
    private int previousPipelineNum = leftPipeline;
    TelemetryManager panelsTelemetry;
    Timer timer;
    PoseBuffer mt1Buffer;
    PoseBuffer mt2Buffer;
    ButtonToggle x1;
    RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection;

    IMU imu;

    @Override
    public void init() {

        timer = new Timer();
        limelight = hardwareMap.get(Limelight3A.class, configName);
        limelight.pipelineSwitch(currentPipeline);//set it to motif pipeline
        telemetry.setMsTransmissionInterval(10);
        limelight.start();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        x1 = new ButtonToggle();
        mt1Buffer = new PoseBuffer();
        mt2Buffer = new PoseBuffer();

        logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)));
    }



    private void createDashboardTelemetry(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        YawPitchRollAngles imuAngles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(imuAngles.getYaw(AngleUnit.DEGREES));


        createDashboardTelemetry();

        timer.updateTime();

        telemetry.addData("Update rate", 1/ timer.getDeltaTime());
        telemetry.addData("Yaw(Deg): ", imuAngles.getYaw(AngleUnit.DEGREES));

        if(previousPipelineNum != currentPipeline){
            limelight.pipelineSwitch(currentPipeline);
            telemetry.addLine("Swapping to pipeline " + currentPipeline);
        }

        if(x1.update(gamepad1.x)) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                Pose3D megaTag1Pose = result.getBotpose();
                Position averaged1Pose = mt1Buffer.update(megaTag1Pose);
                averaged1Pose = averaged1Pose.toUnit(DistanceUnit.INCH);

                Pose3D megaTag2Pose = result.getBotpose_MT2();
                Position averaged2Pose = mt2Buffer.update(megaTag2Pose);
                averaged2Pose = averaged2Pose.toUnit(DistanceUnit.INCH);

                telemetry.addData("MT1Pose: ", megaTag1Pose.toString());
                telemetry.addData("MT1Position(Buffered): ", averaged1Pose.toString());
                telemetry.addData("MT2Pose: ", megaTag2Pose.toString());
                telemetry.addData("MT2Position(Buffered): ", averaged2Pose.toString());

                List<LLResultTypes.FiducialResult> list = result.getFiducialResults();

                telemetry.addLine("--------------");
                for (LLResultTypes.FiducialResult f : list) {
                    Position camPoseTarget = f.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
                    updatePanelsTelemetry(f, camPoseTarget, megaTag1Pose.getPosition(), averaged1Pose);

                    telemetry.addData("April Tag ID", f.getFiducialId());
//                    telemetry.addData("Corners",  f.getTargetCorners().toString());
                    telemetry.addData("Camera Pos X",  camPoseTarget.x);
                    telemetry.addData("Camera Pos Y",  camPoseTarget.y);
                    telemetry.addData("Camera Pos Z",  camPoseTarget.z);
                    telemetry.addData("Heading(Deg) Field",  f.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.DEGREES));
                    telemetry.addData("X Angle",  f.getTargetXDegrees());
                    telemetry.addData("Y Angle", f.getTargetYDegrees());

                }

            }

            previousPipelineNum = currentPipeline;
            telemetry.update();

        }

    }


    private void updatePanelsTelemetry(LLResultTypes.FiducialResult f, Position camPoseTarget, Position megaTag1, Position averagedPose){
        panelsTelemetry.addData("April Tag ID",  f.getFiducialId());
        panelsTelemetry.addData("Corners",  f.getTargetCorners().toString());
        panelsTelemetry.addData("Camera Pos X",  camPoseTarget.x);
        panelsTelemetry.addData("Camera Pos Y",  camPoseTarget.y);
        panelsTelemetry.addData("Camera Pos Z",  camPoseTarget.z);
        panelsTelemetry.addData("X Angle", f.getTargetXDegrees());
        panelsTelemetry.addData("Y Angle",  f.getTargetYDegrees());

        panelsTelemetry.addData("MegaTagLocalizationPose X", megaTag1.x);
        panelsTelemetry.addData("MegaTagLocalizationPose Y", megaTag1.y);
        panelsTelemetry.addData("MegaTagLocalizationPose Z", megaTag1.z);

        panelsTelemetry.addData("Averaged MegaPose X", averagedPose.x);
        panelsTelemetry.addData("Averaged MegaPose Y", averagedPose.y);
        panelsTelemetry.addData("Averaged MegaPose Z", averagedPose.z);

        panelsTelemetry.update();
    }

}
