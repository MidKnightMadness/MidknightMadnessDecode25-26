package org.firstinspires.ftc.teamcode.commands;

import android.os.Environment;

import com.pedropathing.geometry.Pose;
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

public class PoseWriteCommand extends CommandBase {
    double maxTimeMs;
    String xFileName = "robot_x.txt";
    String yFileName = "robot_y.txt";
    String headingFileName = "robot_heading.txt";
    String directoryName = "competition";
    FileWriter xFileWriter;
    FileWriter yFileWriter;
    FileWriter headingFileWriter;
    File xFile;
    File yFile;
    File headingFile;
    boolean finishedWriting = false;
    Timer timer;
    Pose pose;
    public PoseWriteCommand(Pose pose, double timeMs){
        timer = new Timer();
        this.pose = pose;
        this.maxTimeMs = timeMs;
        xFile = createFile(xFileName, directoryName);
        yFile = createFile(yFileName, directoryName);
        headingFile = createFile(headingFileName, directoryName);
        try {
            xFileWriter = new FileWriter(xFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            yFileWriter = new FileWriter(yFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            headingFileWriter = new FileWriter(headingFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    public void execute() {
        double currentTime = timer.getTime();
        if(currentTime < maxTimeMs && !finishedWriting){
            String xLine = String.format("%.4f", pose.getX());
            String yLine = String.format("%.4f", pose.getY());
            String headingLine = String.format("%.4f", pose.getHeading());
            writeToFile(xFileWriter, xLine);
            closeFileWriter(xFileWriter);

            writeToFile(yFileWriter, yLine);
            closeFileWriter(yFileWriter);

            writeToFile(headingFileWriter, headingLine);
            closeFileWriter(headingFileWriter);
        }
    }



    @Override
    public boolean isFinished() {
        return (timer.getTime() > maxTimeMs);
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
