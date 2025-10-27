package org.firstinspires.ftc.teamcode.motif;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Configurable
@Config
@Autonomous(group = "Motif" , name = "Write")
public class MotifAutonomous extends OpMode{
    private Limelight3A limelight;
    public static String configName = "limelight";
    private int motifPipeline = 1;
    TelemetryManager panelsTelemetry;

    Timer timer;
    boolean finishedWriting = false;
    static double detectionMaxTime = 20;

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
           RobotLog.ee("Log", "Error instantiating file writer of file: " + e.getMessage());
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


    private void writeToFile(FileWriter fileWriter, String s){
        try {
            fileWriter.write(s);
            fileWriter.flush();
        } catch (IOException e) {
            RobotLog.ee("Log", "No file writer detected: " + e.getMessage());
        }
    }

    private void closeFileWriter(FileWriter fileWriter){
        try {
            fileWriter.close();
        } catch (IOException e) {
            RobotLog.ee("Log", "Cannot close file writer: " + e.getMessage());
        }
    }

    @Override
    public void init_loop(){

        createDashboardTelemetry();

        telemetry.addData("Finished writing:", finishedWriting);
        telemetry.addData("Current Time:", timer.updateTime());
        telemetry.addData("Update Rate:", 1 / timer.getDeltaTime());
        if(!finishedWriting && timer.updateTime()< detectionMaxTime) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
                ArrayList<Integer> aprilTagsIDDetected = new ArrayList<>();

                for (LLResultTypes.FiducialResult f : list) {
                    Position camPoseTarget = f.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);

                    updatePanelsTelemetry(f, camPoseTarget);
                    telemetry.addLine("April Tag ID" + f.getFiducialId());
                    telemetry.addLine("Corners" + f.getTargetCorners().toString());
                    telemetry.addLine("Camera Pos" + camPoseTarget.toString());
                    telemetry.addLine("Heading(Deg) Field" + f.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.DEGREES));
                    telemetry.addLine("X Angle" + f.getTargetXDegrees());
                    telemetry.addLine("Y Angle" + f.getTargetYDegrees());
                    aprilTagsIDDetected.add(f.getFiducialId());
                }

                writeToFile(fileWriter, aprilTagsIDDetected.toString());
                telemetry.update();
                finishedWriting = true;
            }
        }

        if(timer.updateTime() > detectionMaxTime || finishedWriting) {
            closeFileWriter(fileWriter);
        }
    }

    private void createDashboardTelemetry(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    private void updatePanelsTelemetry(LLResultTypes.FiducialResult f, Position camPoseTarget){
        panelsTelemetry.addLine("April Tag ID" +  f.getFiducialId());
        panelsTelemetry.addLine("Corners" +  f.getTargetCorners().toString());
        panelsTelemetry.addLine("Camera Pos" +  camPoseTarget.toString());
        panelsTelemetry.addLine("X Angle" + f.getTargetXDegrees());
        panelsTelemetry.addLine("Y Angle" + f.getTargetYDegrees());
        panelsTelemetry.update();
    }

    @Override
    public void loop() {

    }
}
