package org.firstinspires.ftc.teamcode.main.autonomous;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.readwrite.PoseWriteCommand;
import org.firstinspires.ftc.teamcode.commands.readwrite.SideWriteCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.tests.opModes.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Config
@Configurable
public class BaseAuto extends CommandOpMode {
    protected Follower follower;
    protected Timer gameTimer;
    Pose startPose;

    protected Limelight3A limelight;
    protected Spindexer spindexer;
    protected TwoWheelShooter shooter;
    protected Intake intake;
    boolean prevVisionComplete = false;

    protected AprilTagWebcam arducam;
    public static double maxTimeMs = 29300;
    public static double maxWritePoseTimeMs = 300;
    public static double maxSideWriteTimeMs = 300;
    boolean stopEnd = false;
    ShootSide side;
    boolean postMotif = false;
    boolean gameTimerStarted = false;
    Command preMotifSeq;
    boolean preMotifNull = false;

    @Override
    public void initialize() {

//        CommandScheduler.getInstance().setBulkReading(
//                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
//        );
        CommandScheduler.getInstance().cancelAll();
        super.reset();

        gameTimer = new Timer();




        startPose = getStartPose();
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        follower.setPose(startPose);
//
//        dashboard = FtcDashboard.getInstance();
//        dashboardPacket = new TelemetryPacket();
//
//        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
//        graphManager = PanelsGraph.INSTANCE.getManager();
        initializeMechanisms();
        buildPaths();
        setupVision();

        preMotifSeq = preMotifSequence();
        if(preMotifSeq != null) {
            schedule(preMotifSeq);
        } else{
            preMotifNull = true;
        }

    }




    protected void initializeMechanisms() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);

    }

//    protected BallColor[] getStartBallColors(){
//        return null;
//    }

    protected ShootSide getSide(){
        return ShootSide.LEFT;
    }

    @Override
    public void run(){
        super.run();
        if(!gameTimerStarted){
            gameTimer.restart();
            gameTimerStarted = true;
        }
        update();
        if(preMotifNull || (!prevVisionComplete && isVisionComplete())){
//            if(postMotifSequence() != null) {
            if(!preMotifNull) {
                preMotifSeq.cancel();
                follower.breakFollowing();
            }
            schedule(postMotifSequence());
            prevVisionComplete = true;
            preMotifNull = false;
        }

     //   if(postMotifSequence().isFinished()){
//            if(goToIntakeLine()!= null){
//                schedule(goToIntakeLine());
//            }
    //    }
//        if (timer.getTime() >= maxTimeMs) requestOpModeStop();
//        writeValues();
        updateTelemetry();
    }


    String directoryName = "competition";
    FileWriter xFileWriter;
    FileWriter yFileWriter;
    FileWriter headingFileWriter;
    File xFile;
    File yFile;
    File headingFile;

    String xFileName = "robot_x.txt";
    String yFileName = "robot_y.txt";
    String headingFileName = "robot_heading.txt";
    public void update(){

    }

    @Override
    public void end(){
//        writeMotif();
//        schedule(new DeferredCommand(() ->
//                new SequentialCommandGroup(
//                        new InstantCommand(()-> follower.update()),
//                        new WaitCommand(200),
//                        new ParallelCommandGroup(
//                                new PoseWriteCommand(follower.getPose(), maxWritePoseTimeMs),
//                                new SideWriteCommand(getSide(), maxSideWriteTimeMs))), null));
//        stopEnd = true;

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
            follower.update();
            Pose pose = follower.getPose();
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

    public void writeValues() {
        if(gameTimer.getTime() >= maxTimeMs && !stopEnd) {
            CommandScheduler.getInstance().cancelAll();
            writeMotif();
            schedule(new DeferredCommand(() ->
                new SequentialCommandGroup(
                        new InstantCommand(()-> follower.update()),
                        new WaitCommand(200),
                new ParallelCommandGroup(
                    new PoseWriteCommand(follower.getPose(), maxWritePoseTimeMs),
                    new SideWriteCommand(getSide(), maxSideWriteTimeMs))), null));
            stopEnd = true;
        }
    }

    public void writeMotif(){
    }

//    public Command goToIntakeLine(){
//        return null;
//    }


    protected Command postMotifSequence() {
        return null;
    }

    protected Command preMotifSequence() {
        return null;
    }

    protected boolean isVisionComplete() {
        return true;
    }

    protected void setupVision() {
    }

    protected void buildPaths() {
    }

    protected void updateTelemetry(){
    }

    protected Pose getStartPose() {
        return null;
    }
}
