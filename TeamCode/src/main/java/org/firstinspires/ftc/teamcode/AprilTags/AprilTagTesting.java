package org.firstinspires.ftc.teamcode.AprilTags;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Timer;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Vision", name = "AprilTagTesting")
@Configurable
public class AprilTagTesting extends OpMode{

    private Limelight3A limelight;
    public static String configName = "limelight";
    private double leftPipeline = 0;
    private double motifPipeline = 1;
    private double rightPipeline = 2;

    public static int currentPipeline = 1;
    private int previousPipelineNum = 1;
    public static boolean repeatTest = false;
    Timer timer;


    @Override
    public void init() {

        timer = new Timer();
        limelight = hardwareMap.get(Limelight3A.class, configName);
        telemetry.setMsTransmissionInterval(5);
        limelight.pipelineSwitch(currentPipeline);//set it to motif pipeline
        limelight.start();

    }




    @Override
    public void loop() {

        timer.updateTime();
        telemetry.addData("Update rate", 1/ timer.getDeltaTime());


        if(previousPipelineNum != currentPipeline){
            limelight.pipelineSwitch(currentPipeline);
            telemetry.addLine("Swapping to pipeline " + currentPipeline);
        }

        if(!repeatTest) {
            repeatTest = true;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
                for (LLResultTypes.FiducialResult f : list) {
                    telemetry.addLine("April Tag ID" +  f.getFiducialId());
                    telemetry.addLine("Corners" +  f.getTargetCorners().toString());
                    telemetry.addLine("Camera Pos" +  f.getCameraPoseTargetSpace().toString());
                    telemetry.addLine("Robot Pos Field" + f.getRobotPoseFieldSpace().toString());
                    telemetry.addLine("Robot Pos Target" + f.getRobotPoseTargetSpace().toString());
                    telemetry.addLine("X Angle" + f.getTargetXDegrees());
                    telemetry.addLine("Y Angle" + f.getTargetYDegrees());
                }
            }
        }

        previousPipelineNum = currentPipeline;
    }
}
