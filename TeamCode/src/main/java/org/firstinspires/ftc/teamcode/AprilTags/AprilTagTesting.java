package org.firstinspires.ftc.teamcode.AprilTags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Util.ButtonToggle;
import org.firstinspires.ftc.teamcode.Util.PoseBuffer;
import org.firstinspires.ftc.teamcode.Util.Timer;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Vision", name = "AprilTagTesting")
@Configurable
public class AprilTagTesting extends OpMode{

    private Limelight3A limelight;
    public static String configName = "limelight";
    private static int leftPipeline = 0;
    private static int motifPipeline = 1;
    private static int rightPipeline = 2;

    public static int currentPipeline = leftPipeline;
    private int previousPipelineNum = leftPipeline;
    TelemetryManager panelsTelemetry;
    Timer timer;
    PoseBuffer buffer;

    ButtonToggle x1;

    @Override
    public void init() {

        timer = new Timer();
        limelight = hardwareMap.get(Limelight3A.class, configName);
        limelight.pipelineSwitch(currentPipeline);//set it to motif pipeline
        telemetry.setMsTransmissionInterval(10);
        limelight.start();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        x1 = new ButtonToggle();
        buffer = new PoseBuffer();
    }



    private void createDashboardTelemetry(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        createDashboardTelemetry();

        timer.updateTime();
        telemetry.addData("Update rate", 1/ timer.getDeltaTime());


        if(previousPipelineNum != currentPipeline){
            limelight.pipelineSwitch(currentPipeline);
            telemetry.addLine("Swapping to pipeline " + currentPipeline);
        }

        if(x1.update(gamepad1.x)) {
            telemetry.update();
            LLResult result = limelight.getLatestResult();

            Pose3D megaTag1Pose = result.getBotpose();
            Position averagedPose = buffer.update(megaTag1Pose);


            telemetry.addData("MegaTagLocalizationPose X", megaTag1Pose.getPosition().x);
            telemetry.addData("MegaTagLocalizationPose Y", megaTag1Pose.getPosition().y);
            telemetry.addData("MegaTagLocalizationPose Z", megaTag1Pose.getPosition().z);
            telemetry.addData("Averaged MegaPose X", averagedPose.x);
            telemetry.addData("Averaged MegaPose Y", averagedPose.y);
            telemetry.addData("Averaged MegaPose Z", averagedPose.z);

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
                for (LLResultTypes.FiducialResult f : list) {
                    Position camPoseTarget = f.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
                    updatePanelsTelemetry(f, camPoseTarget, megaTag1Pose.getPosition(), averagedPose);

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
