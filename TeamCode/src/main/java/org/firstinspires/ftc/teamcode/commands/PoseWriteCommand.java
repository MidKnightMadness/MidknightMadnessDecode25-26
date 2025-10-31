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
    String fileName = "robot_pose.txt";
    String directoryName = "competition";
    FileWriter fileWriter;
    File file;
    boolean finishedWriting = false;
    Timer timer;
    Pose pose;
    public PoseWriteCommand(Pose pose, double timeMs){
        timer = new Timer();
        this.pose = pose;
        this.maxTimeMs = timeMs;
        file = createFile(fileName, directoryName);

        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            RobotLog.ee("Log", "Error instantiating file writer of file: " + e.getMessage());
        }
    }

    @Override
    public void execute() {
        double currentTime = timer.getTime();
        if(currentTime < maxTimeMs && !finishedWriting){
            String line = String.format("%.4f,%.4f,%.4f", pose.getX(), pose.getY(), pose.getHeading());
            writeToFile(fileWriter, line);
            finishedWriting = true;
            closeFileWriter(fileWriter);
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
