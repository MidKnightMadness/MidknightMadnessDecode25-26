

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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Config
@Configurable
@Autonomous(name = "3 Close Right", group = "Competition")
public class ThreeBallCloseRightAuto extends BaseAuto {
    public static double motifDetectionTimeMs = 5000;
    int startPipeline = 1;
    public static Pose startPose = new Pose(118.7, 130, Math.toRadians(43));
    public static Pose motifDetectionPose = new Pose(87, 94, Math.toRadians(100));
    public static Pose shootPose = new Pose(87, 94, Math.toRadians(230));
    public static Pose leavePose = new Pose(85, 67, Math.toRadians(0));
    PathChain toMotifPath;
    PathChain toShootingPath;
    PathChain leaveBasePath;
    MotifEnums.Motif motifPattern;
    MotifWriteCommand motifCommand;

    public static ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose;

    Command firstPath;
    double speed;
    double acc;
    public static long waitTime = 1000;
    public static double pathDistThresholdMin = 0.5;
    public static double headingError = 0.025;
    public static double timeOutConstraint = 200;
    BallColor[] startBallColors = new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    @Override
    protected Pose getStartPose(){
             return startPose;
         }
    @Override
    protected void setupVision(){
        limelight.pipelineSwitch(startPipeline);
        limelight.start();
    }

    @Override
    protected BallColor[] getStartBallColors(){
        return startBallColors;
    }



    @Override
    protected void initializeMechanisms() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        if(getStartBallColors() != null){
            spindexer = new Spindexer(hardwareMap, true).setBallColors(getStartBallColors());
        }
        else{
            spindexer = new Spindexer(hardwareMap, true).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        }

        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);

        shooter.low.setVeloCoefficients(pidBotGainsShooter[0], pidBotGainsShooter[1], pidBotGainsShooter[2]);
        shooter.high.setVeloCoefficients(pidTopGainsShooter[0], pidTopGainsShooter[1], pidTopGainsShooter[2]);
        shooter.low.setFeedforwardCoefficients(kBotGainsShooter[0], kBotGainsShooter[1], kBotGainsShooter[2]);
        shooter.high.setFeedforwardCoefficients(kTopGainsShooter[0], kTopGainsShooter[1], kTopGainsShooter[2]);


        shooter.low.resetEncoder();
        shooter.high.resetEncoder();
    }



    @Override
    protected ShootSide getSide(){
             return shootSide;
         }

    @Override
    protected void buildPaths(){
        toMotifPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, motifDetectionPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading())
                .setHeadingConstraint(headingError)
                .setTimeoutConstraint(timeOutConstraint)
                .setTranslationalConstraint(pathDistThresholdMin)
                .build();
        toShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(motifDetectionPose, shootPose))
                .setLinearHeadingInterpolation(motifDetectionPose.getHeading(), shootPose.getHeading())
                .setHeadingConstraint(headingError)
                .setTranslationalConstraint(pathDistThresholdMin)
                .setTimeoutConstraint(timeOutConstraint)
                .build();
        leaveBasePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .setHeadingConstraint(headingError)
                .setTranslationalConstraint(pathDistThresholdMin)
                .setTimeoutConstraint(timeOutConstraint)
                .build();
         }
     @Override
     protected boolean isVisionComplete(){
        motifPattern = motifCommand.getDetected();
        if(motifCommand.isFinished()){
            return true;
        }
        return false;
    }

     @Override
     protected Command preMotifSequence(){
         motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
         firstPath = new FollowPathCommand(follower, toMotifPath).setGlobalMaxPower(0.5);
         return new SequentialCommandGroup(
                 new InstantCommand(() -> spindexer.setBallColors(startBallColors)),
                 firstPath,
                 motifCommand
         );

     }

     @Override
     protected Command postMotifSequence(){
         limelight.stop();//temporarily turn it off to hand to localizer
         return new SequentialCommandGroup(
                 new WaitCommand(waitTime),
                 new FollowPathCommand(follower, toShootingPath, true),
                 new WaitCommand(waitTime),
                 new ShootSeqCommand(spindexer, shooter, spindexer.getOptimalSequence(motifPattern), follower, shootSide, false, TwoWheelShooter.ShootDist.Close, true),
                 new WaitCommand(waitTime),
                 new FollowPathCommand(follower, leaveBasePath, true)
         );

     }

     @Override
     protected void updateTelemetry(){
         follower.update();
         currentPose = follower.getPose();
         gameTimer.getTime();
         addBooleanToTelem("First path is busy", firstPath.isFinished());
         addBooleanToTelem("Motif Busy", ConstantsBot.motifIsBusy);
         addStringToTelem("Motif timer", String.valueOf(motifCommand.timer.getTime()));
         addStringToTelem("Motif Pattern", String.valueOf(motifPattern));
         addToTelemGraph("Current Time", gameTimer.getTime());
         addToTelemGraph("Update Rate", 1/gameTimer.getDeltaTime());
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




