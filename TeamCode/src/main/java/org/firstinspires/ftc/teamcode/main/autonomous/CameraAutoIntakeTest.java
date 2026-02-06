package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand3;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.tests.camera.CamCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
@Config
@Configurable
@Autonomous(name = "Camera Auto Intake Test", group = "Test")
public class CameraAutoIntakeTest extends BaseAuto {
    int limelightPipelineStart = 3;//CHANGE THIS
    public static Pose startPose = new Pose(96, 8, Math.toRadians(90));//CHANGE THIS
    public static Pose startDetectPose = new Pose(110, 10, Math.toRadians(0));
    public static Pose strafeOnePose = new Pose(110, 20, Math.toRadians(0));
    public static Pose strafeTwoPose = new Pose(110, 30, Math.toRadians(0));
    public static Pose strafeThreePose = new Pose(110, 40, Math.toRadians(0));
    public static Pose strafeFourPose = new Pose(110, 50, Math.toRadians(0));
    CamCommand cam;

    public static double robotSpeed = 0.5;
    public static double targetX = 130;


    public static boolean drive = true;
    public static double inBetweenTime = 200;
    public static long detectTime = 2000;
    boolean ballDetected = false;
    public static double intakePower = 0.8;

    public static double strafePower = 0.5;
    PathChain driveToPath;
    Pose targetPose;
    BallColor[] ballColors;
    public static boolean useBulkMode = false;
    public static long waitBetweenStrafe = 200;
    public static long waitIntakeTimeout = 200;
    public static double strafeDistancePer = 10;

    PathChain driveToStartPath;

    PathChain strafe1Path;
    PathChain strafe2Path;
    PathChain strafe3Path;
    PathChain strafe4Path;

    @Override
    protected Pose getStartPose(){
        return startPose;
    }

    @Override
    protected void initializeMechanisms() {
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);//init intak
        spindexer = new Spindexer(hardwareMap, true).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}).initAngle();//init spindexer
        if(useBulkMode) {
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
            );
        } else{
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
            );
        }
        register(intake, spindexer);
    }

    @Override
    protected void setupVision() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.pipelineSwitch(limelightPipelineStart);
        limelight.start();
    }

    @Override
    protected Command preMotifSequence(){
       return null;
    }

    @Override
    protected boolean isVisionComplete(){
        return true;
    }

    @Override
    protected Command postMotifSequence() {
        cam = new CamCommand(limelight, follower);
        return new SequentialCommandGroup(
                driveToStartViewPosition(),
                new WaitCommand(200),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            cam,
                            new InstantCommand(()-> camDetect = true)
                        ),
                        new SequentialCommandGroup(
                            strafeNumber(1),
                            new WaitCommand(waitBetweenStrafe),
                            strafeNumber(2),
                            new WaitCommand(waitBetweenStrafe),
                            strafeNumber(3),
                            new WaitCommand(waitBetweenStrafe),
                            strafeNumber(4)
                        )
                ),
//                new InstantCommand(() -> buildDrivePath()),
                intakeBall()
        );
    }


    public Command driveToStartViewPosition(){
        return new FollowPathCommand(follower, driveToStartPath, true, robotSpeed);
    }

    @Override
    protected void buildPaths() {
        driveToStartPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startDetectPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startDetectPose.getHeading())
                .build();
        strafe1Path = follower.pathBuilder()
                .addPath(new BezierLine(startDetectPose, strafeOnePose))
                .setLinearHeadingInterpolation(startDetectPose.getHeading(), strafeOnePose.getHeading())
                .build();
        strafe2Path = follower.pathBuilder()
                .addPath(new BezierLine(strafeOnePose, strafeTwoPose))
                .setLinearHeadingInterpolation(strafeOnePose.getHeading(), strafeTwoPose.getHeading())
                .build();
        strafe3Path = follower.pathBuilder()
                .addPath(new BezierLine(strafeTwoPose, strafeThreePose))
                .setLinearHeadingInterpolation(strafeTwoPose.getHeading(), strafeThreePose.getHeading())
                .build();
        strafe4Path = follower.pathBuilder()
                .addPath(new BezierLine(strafeThreePose, strafeFourPose))
                .setLinearHeadingInterpolation(strafeThreePose.getHeading(), strafeFourPose.getHeading())
                .build();

    }

    public Pose offset(Pose currPose, double strafeDistance){
        return new Pose(currPose.getX(), currPose.getY() - strafeDistance, currPose.getHeading());
    }

    boolean setPath = false;
    boolean camDetect;

    private Command buildBallPathSequence(CamCommand camera) {
        /*
        ParallelCommandGroup parallel = new ParallelCommandGroup();
        //parallel.addCommands(new AutoIntakeCommand3(spindexer, intake, 0.5, 0.5, false, hardwareMap));
        if(camera.finalBallList != null && camera.finalBallList.size() > 0) {
            parallel.addCommands(new FollowPathCommand(follower, camera.getList(follower)));
        }
        */
        if(camera.getList(follower, targetX)!= null && camera.getList(follower, targetX).getPath(0)!= null) {
            return new FollowPathCommand(follower, camera.getList(follower, targetX).getPath(0), true, intakePower);
        } else{
            return new InstantCommand();
        }

    }
    public PathChain buildDrivePath(boolean camDetect){
        follower.update();
//        if(camDetect) {
            targetPose = new Pose(0, 0, 0);
            driveToPath = cam.getList(follower, targetX);
            setPath = true;
//        }
//        else{
//            Pose currPose = follower.getPose();
//            targetPose = new Pose(targetX, currPose.getY(), currPose.getHeading());
//            driveToPath = follower.pathBuilder()
//                    .addPath(new BezierLine(currPose, targetPose))
//                    .setLinearHeadingInterpolation(currPose.getHeading(), targetPose.getHeading())
//                    .build();
//            setPath = false;
//        }
        return driveToPath;
    }



    @Override
    public void update(){
//        if(cam != null){
//            ballDetected = (cam.getFinalGlobalPoseList() != null && cam.getFinalGlobalPoseList().size() != 0);
//        }
//      ballColors = spindexer.getBallColors();

    }

    public Command intakeBall(){

//        return new SequentialCommandGroup(
//                buildDrivePath(),
       return new ParallelRaceGroup(
            new AutoIntakeCommand3(spindexer, intake, intakePower, inBetweenTime, true, hardwareMap),
            new SequentialCommandGroup(
                new WaitCommand(1000),
//                new FollowPathCommand(follower, buildDrivePath(camDetect), true, robotSpeed),
                buildBallPathSequence(cam),
                new WaitCommand(waitIntakeTimeout)
            )
        );
    }


//    private Command strafeLeft(double distance){
//        follower.update();
//        Pose botPose = follower.getPose();
//        Pose leftPose = new Pose(botPose.getX(), botPose.getY() + distance, botPose.getHeading());
//        driveToPath = follower.pathBuilder()
//                .addPath(new BezierLine(botPose, leftPose))
//                .setLinearHeadingInterpolation(botPose.getHeading(), leftPose.getHeading())
//                .build();
//        return new FollowPathCommand(follower, driveToPath, strafePower);//this moves left
//
//    }
    private Command strafeNumber(int numb){//start at 1
        PathChain pathChain = (numb == 1) ? strafe1Path : (numb == 2) ? strafe2Path : (numb == 3) ? strafe3Path : strafe4Path;
        return new FollowPathCommand(follower, pathChain, true, strafePower);
    }

    private Command goBack(){
        Pose botPose = follower.getPose();
        Pose backPose = new Pose(92, botPose.getY(), botPose.getHeading());
        return new FollowPathCommand(follower, backPose, 0.5);
    }


    @Override
    protected void updateTelemetry(){

        telemetry.addData("Camera Path", setPath);

        telemetry.addData("Update Rate", 1000.0 / gameTimer.getDeltaTime());
        telemetry.addData("Ball Detected", camDetect);
        if(cam != null) {
            telemetry.addData("CamCommand finished", cam.isFinished());
        }
        telemetry.addData("Robot position", follower.getPose());
        telemetry.update();
        if(cam == null || cam.getList(follower, targetX) == null || cam.getList(follower, targetX).getPath(0)== null) {
            telemetry.addData("First", "null");
        }
        else{
            for(Pose pose : cam.getFinalBallList()){
                telemetry.addData("pose", pose);
            }

            telemetry.addLine("GLOBAL");
            for(Pose pose : cam.getFinalGlobalPoseList()){
                telemetry.addData("pose", pose);
            }
        }

        if(cam != null && cam.getList(follower, targetX)== null){
            telemetry.addLine("First Null");
        } else if(cam != null && cam.getList(follower, targetX)== null){
            telemetry.addLine("First Null");
        }

        if(targetPose!= null){
            telemetry.addData("Target Pose", targetPose);
        }
        if(ballColors != null) {
            telemetry.addData("Ball Color 1", ballColors[0]);
            telemetry.addData("Ball Color 1", ballColors[1]);
            telemetry.addData("Ball Color 1", ballColors[2]);
        }
        follower.update();
    }
}
