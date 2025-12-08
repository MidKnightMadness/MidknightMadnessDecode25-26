
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
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

 @Config
 @Configurable
 @Autonomous(name = "3 Back Right", group = "Competition")
 public class ThreeBallBackRightAuto extends BaseAuto {
     public static double motifDetectionTimeMs = 5000;
     int startPipeline = 1;

     public static Pose startPose = new Pose(88, 8, Math.toRadians(90));
     public static Pose shootPose = new Pose(84, 17, Math.toRadians(248));
     public static Pose leavePose = new Pose(86, 38, Math.toRadians(0));
     PathChain toShootingPath;
     PathChain leaveBasePath;
     MotifEnums.Motif motifPattern;
     MotifWriteCommand motifCommand;

     public static ShootSide shootSide = ShootSide.RIGHT;
     Pose currentPose;

     double speed;
     double acc;

     public static long waitTime = 1000;
     public static double pathDistThresholdMax = 0.5;
     public static double headingErrorMax = 0.025;
     double timeOutConstraint = 200;

     public static BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};


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
     protected void buildPaths(){
         toShootingPath = follower.pathBuilder()
                 .addPath(new BezierLine(startPose, shootPose))
                 .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                 .setHeadingConstraint(headingErrorMax)
                 .setTimeoutConstraint(timeOutConstraint)
                 .setTranslationalConstraint(pathDistThresholdMax)
                 .build();
         leaveBasePath = follower.pathBuilder()
                 .addPath(new BezierLine(shootPose, leavePose))
                 .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                 .setHeadingConstraint(headingErrorMax)
                 .setTimeoutConstraint(timeOutConstraint)
                 .setTranslationalConstraint(pathDistThresholdMax)
                 .build();
     }


     @Override
     protected boolean isVisionComplete(){
         motifPattern = motifCommand.getDetected();
         if(motifPattern != MotifEnums.Motif.NONE){
             return true;
         }
         return false;
     }

     @Override
     protected Command preMotifSequence(){
         motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
         return new SequentialCommandGroup(
                 new InstantCommand(() -> spindexer.setBallColors(startBallColors)),
                 motifCommand
         );

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
     protected Command postMotifSequence(){
         return new SequentialCommandGroup(
                 new FollowPathCommand(follower, toShootingPath, true).setGlobalMaxPower(0.8),
                 new WaitCommand(waitTime),
                 new ShootSeqCommand(spindexer, shooter, spindexer.getOptimalSequence(motifPattern), follower, shootSide, false, TwoWheelShooter.ShootDist.Close, true),
                 new WaitCommand(waitTime),
                 new FollowPathCommand(follower, leaveBasePath, true)
         );
     }

     protected void updateTelemetry(){
         follower.update();
         currentPose = follower.getPose();
         telemetry.addData("Current Time", gameTimer.getTime());
         telemetry.addData("Motif Pattern", String.valueOf(motifPattern));
         telemetry.addData("Update Rate", 1/gameTimer.getDeltaTime());
         telemetry.addData("Pose(X)", currentPose.getX());
         telemetry.addData("Pose(Y)", currentPose.getY());
         telemetry.addData("Pose(Heading)", currentPose.getHeading());
         telemetry.addData("Speed(in/s)", (speed != 0 ? speed : 0));
         telemetry.addData("Acc(in/s^2)", (acc != 0 ? acc : 0));
         telemetry.update();
//         graphManager.update();;
//         telemetryManager.update();;
     }


    @Override
    protected ShootSide getSide(){
        return shootSide;
    }

     public void addStringToTelem(String s, String o){
         telemetry.addLine(s + o);
     }
     public void addToTelemGraph(String s, Number o){
         telemetry.addData(s, o);
         telemetryManager.addData(s, o);
         graphManager.addData(s, o);
     }


 }

