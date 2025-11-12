package org.firstinspires.ftc.teamcode.commands;

import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class SideWriteCommand extends CommandBase {
    double maxTimeMs;
    String sideFileName = "side.txt";
    String directoryName = "competition";
    FileWriter sideFileWriter;
    File sideFile;
    boolean finishedWriting = false;
    Timer timer;
    ShootSide side;
    String outputString;

    public SideWriteCommand(ShootSide shootSide, double timeMs){
        timer = new Timer();
        this.side = shootSide;
        this.maxTimeMs = timeMs;
        sideFile = createFile(sideFileName, directoryName);
        if(shootSide == ShootSide.LEFT){
            outputString = "Left";
        }
        else{
            outputString = "Right";
        }
        try {
            sideFileWriter = new FileWriter(sideFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    public void execute() {
        double currentTime = timer.getTime();
        if(currentTime < maxTimeMs && !finishedWriting){
            writeToFile(sideFileWriter, outputString);
            closeFileWriter(sideFileWriter);
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
