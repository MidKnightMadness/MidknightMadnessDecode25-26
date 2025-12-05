package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.IntakeSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeTimeCommand;
import org.firstinspires.ftc.teamcode.commands.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Config
@Configurable
@Autonomous(name = "12 Back Left", group = "Competition")
public class TwelveBallCloseRightAuto extends BaseAuto {
    public static double motifDetectionTimeMs = 5000;
    int startPipeline = 1;
    public static Pose startPose = new Pose(118, 130, Math.toRadians(37));
    public static Pose motifDetectionPose = new Pose(87, 94, Math.toRadians(100));
    public static Pose shootPose = new Pose(87, 94, Math.toRadians(210));
    public static Pose parkPose = new Pose(114, 94, Math.toRadians(210));
    public static Pose openGatePose = new Pose(129, 70, Math.toRadians(180));
    public static Pose intakeOnePose = new Pose(107, 84, Math.toRadians(0));
    public static Pose intakeTwoPose = new Pose(107, 60, Math.toRadians(0));
    public static Pose intakeThreePose = new Pose(101, 36, Math.toRadians(0));
    public static double intakeDistForward = 14;
    PathChain toMotifPath;
    MotifEnums.Motif motifPattern;
    MotifWriteCommand motifCommand;

    ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose;

    Command firstPath;
    double speed;
    double acc;

    public static double pathDistThresholdMin = 1;
    public static double headingError = 0.03;
    public static double timeOutConstraint = 500;
    public static double intakeTime = 4000;
    public static BallColor[] ballColorStart = new  BallColor[] {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    @Override
    protected Pose getStartPose(){
        return startPose;
    }

    @Override
    protected void setupVision(){
//        limelight.pipelineSwitch(startPipeline);
////        limelight.start();
    }

    @Override
    protected ShootSide getSide(){
        return shootSide;
    }

    //keep these empty and build the path using follower's current Pose
    @Override
    protected void buildPaths(){
        toMotifPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, motifDetectionPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading())
                .setHeadingConstraint(headingError)
                .setTimeoutConstraint(3000)
                .setTranslationalConstraint(pathDistThresholdMin)
                .build();
    }


    @Override
    protected boolean isVisionComplete(){
//        motifPattern = motifCommand.getDetected();
//        if(motifCommand.isFinished()){
//            ConstantsBot.motifIsBusy = false;
//            return true;
//        }
        return firstPath.isFinished();
    }

    @Override
    protected Command preMotifSequence(){
//        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
        firstPath = new FollowPathCommand(follower, toMotifPath).setGlobalMaxPower(0.7);
        return new SequentialCommandGroup(
                firstPath
//                motifCommand
        );

    }
    @Override
    protected void initializeMechanisms() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        spindexer = new Spindexer(hardwareMap);
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);
    }

    @Override
    protected Command postMotifSequence(){
//        limelight.stop();//temporarily turn it off to hand to localizer
        return new SequentialCommandGroup(
                getToShootCommand(),
                //new InstantCommand(() -> spindexer.setBallColors(ballColorStart)),
                //shoot(),
                getToLineNum(1),
                //new ParallelCommandGroup(
                //        new IntakeTimeCommand(intake, intakeTime)
                //),//some intakeCommand
                getToShootCommand(),
                //shoot(),
                getToLineNum(2),
                //shoot()
                getToShootCommand(),
                getToLineNum(3),
                park()



        );

    }

    protected SchedulePathTo park(){
        return new SchedulePathTo(follower, parkPose, headingError, timeOutConstraint, pathDistThresholdMin);
    }
    protected ShootSeqCommand shoot(){
        return new ShootSeqCommand(spindexer, shooter, spindexer.getOptimalSequence(motifPattern), follower, shootSide, false);
    }
    protected SchedulePathTo getToLineNum(int lineNum){
        if(lineNum == 1) return new SchedulePathTo(follower, intakeOnePose, headingError, timeOutConstraint, pathDistThresholdMin);
        else if(lineNum == 2) return new SchedulePathTo(follower, intakeTwoPose, headingError, timeOutConstraint, pathDistThresholdMin);
        return new SchedulePathTo(follower, intakeThreePose, headingError, timeOutConstraint, pathDistThresholdMin);
    }

    protected SchedulePathTo getToShootCommand(){
        return new SchedulePathTo(follower, shootPose, headingError, timeOutConstraint, pathDistThresholdMin);
    }


    protected void updateTelemetry(){
        follower.update();
        currentPose = follower.getPose();
        timer.getTime();
        addBooleanToTelem("First path is busy", firstPath.isFinished());
        addBooleanToTelem("Motif Busy", ConstantsBot.motifIsBusy);
//        addStringToTelem("Motif timer", String.valueOf(motifCommand.timer.getTime()));
        addStringToTelem("Motif Pattern", String.valueOf(motifPattern));
        addToTelemGraph("Current Time", timer.getTime());
        addToTelemGraph("Update Rate", 1/timer.getDeltaTime());
        addToTelemGraph("Pose(X)", currentPose.getX());
        addToTelemGraph("Pose(Y)", currentPose.getY());
        addToTelemGraph("Pose(Heading)", currentPose.getHeading());
        addToTelemGraph("Speed(in/s)", (speed != 0 ? speed : 0));
        addToTelemGraph("Acc(in/s^2)", (acc != 0 ? acc : 0));
        telemetry.update();
        graphManager.update();;
        telemetryManager.update();;
    }



    public void addStringToTelem(String s, String o){
        telemetry.addLine(s + o);
    }
    public void addToTelemGraph(String s, Number o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
    }
    public void addBooleanToTelem(String s, boolean o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
    }




}

