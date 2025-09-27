package org.firstinspires.ftc.teamcode.AprilTags;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.os.Environment;

import org.firstinspires.ftc.teamcode.Util.Timer;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

@Autonomous(group = "Vision", name = "MotifAutonomous")
public class MotifAutonomous extends OpMode{
    private Limelight3A limelight;
    public static String configName = "limelight";
    private int motifPipeline = 1;
    TelemetryManager panelsTelemetry;
    Timer timer;
    boolean finishedWriting = false;
    double detectionMaxTime = 3;

    PrintWriter pw;
    String fileName = "motif_value.txt";
    String directoryName = "Vision";
    File file;
    FileWriter fileWriter;
    @Override
    public void init(){

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        timer = new Timer();

        limelight = hardwareMap.get(Limelight3A.class, configName);
        limelight.pipelineSwitch(motifPipeline);
        limelight.start();

        file = createFile(fileName, directoryName);
        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    private static File createFile(String fileName, String dirName){
        File dir = new File(Environment.getExternalStorageDirectory(), dirName);
        if(!dir.exists()){
            dir.mkdirs();
        }

        File file = new File(dir, fileName);
        return file;
    }


    @Override
    public void init_loop(){

        if(finishedWriting == false && timer.updateTime() < detectionMaxTime) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
                for (LLResultTypes.FiducialResult f : list) {
                    updatePanelsTelemetry(f);
                    telemetry.addLine("April Tag ID" + f.getFiducialId());
                    telemetry.addLine("Corners" + f.getTargetCorners().toString());
                    telemetry.addLine("Camera Pos" + f.getCameraPoseTargetSpace().toString());
                    telemetry.addLine("Robot Pos Field" + f.getRobotPoseFieldSpace().toString());
                    telemetry.addLine("Robot Pos Target" + f.getRobotPoseTargetSpace().toString());
                    telemetry.addLine("X Angle" + f.getTargetXDegrees());
                    telemetry.addLine("Y Angle" + f.getTargetYDegrees());
                    try {
                        fileWriter.write(f.getFiducialId());
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                }
                finishedWriting = true;
            }
        }

        if(timer.updateTime() > detectionMaxTime) {
            try {
                fileWriter.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public void updatePanelsTelemetry(LLResultTypes.FiducialResult f){
        panelsTelemetry.addLine("April Tag ID" +  f.getFiducialId());
        panelsTelemetry.addLine("Corners" +  f.getTargetCorners().toString());
        panelsTelemetry.addLine("Camera Pos" +  f.getCameraPoseTargetSpace().toString());
        panelsTelemetry.addLine("Robot Pos Field" + f.getRobotPoseFieldSpace().toString());
        panelsTelemetry.addLine("Robot Pos Target" + f.getRobotPoseTargetSpace().toString());
        panelsTelemetry.addLine("X Angle" + f.getTargetXDegrees());
        panelsTelemetry.addLine("Y Angle" + f.getTargetYDegrees());
        panelsTelemetry.update();
    }

    @Override
    public void loop() {

    }
}
