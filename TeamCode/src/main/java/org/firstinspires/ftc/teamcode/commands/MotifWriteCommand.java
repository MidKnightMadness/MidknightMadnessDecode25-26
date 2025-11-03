package org.firstinspires.ftc.teamcode.commands;

import android.os.Environment;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.util.Timer;


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;

public class MotifWriteCommand extends CommandBase {
    Limelight3A limelight;
    double maxTimeMs;
    public static int motifPipeline = 1;
    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    Map<Integer, MotifEnums.Motif> idMap = Map.of(
            21, MotifEnums.Motif.GPP,
            22, MotifEnums.Motif.PGP,
            23, MotifEnums.Motif.PPG
    );
    String fileName = "motif_value.txt";
    String directoryName = "competition";
    FileWriter fileWriter;
    File file;
    boolean finishedWriting = false;
    public Timer timer;

    public MotifWriteCommand(Limelight3A limelight, double timeMs){
        this.limelight = limelight;
        this.maxTimeMs = timeMs;
//        limelight.pipelineSwitch(motifPipeline);
//        limelight.start();
        timer = new Timer();


        file = createFile(fileName, directoryName);

        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            RobotLog.ee("Log", "Error instantiating file writer of file: " + e.getMessage());
        }
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double currentTime = timer.getTime();
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
            for (LLResultTypes.FiducialResult item : list) {
                int aprilTagID = item.getFiducialId();
                motifPattern = idMap.getOrDefault(aprilTagID, MotifEnums.Motif.NONE);
                if (motifPattern != MotifEnums.Motif.NONE) break;
            }

//            if(motifPattern != MotifEnums.Motif.NONE) {
//                writeToFile(fileWriter, String.valueOf(aprilTagID));
//                closeFileWriter(fileWriter);
//                finishedWriting = true;
//            }
        }
    }


    public MotifEnums.Motif getDetected(){
        return motifPattern;
    }

    @Override
    public boolean isFinished() {
        return (timer.getTime() > maxTimeMs) || motifPattern != MotifEnums.Motif.NONE;
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


}
