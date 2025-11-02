

 package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.ShootHardcode;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ShootSide;

     @Config
     @Configurable
     @Autonomous(name = "Close Right")
     public class ThreeBallCloseRightAuto extends BaseAuto {
         public static double motifDetectionTimeMs = 5000;
         int startPipeline = 1;
         public static Pose startPose = new Pose(122, 120, Math.toRadians(-135));
         public static Pose motifDetectionPose = new Pose(87, 95, Math.toRadians(115+180));
         public static Pose shootPose = new Pose(85, 85, Math.toRadians(40+180+4));
         public static Pose leavePose = new Pose(110, 65, Math.toRadians(90));
         PathChain toMotifPath;
         PathChain toShootingPath;
         PathChain leaveBasePath;
         MotifEnums.Motif motifPattern;
         MotifWriteCommand motifCommand;

         ShootSide shootSide = ShootSide.LEFT;
         Pose currentPose;

         Command firstPath;
         double speed;
         double acc;

         public static long waitTime = 3000;
         public static double pathDistThresholdMin = 3;
         public static double headingError = 0.3;
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
         protected ShootSide getSide(){
             return shootSide;
         }
         @Override
         protected void buildPaths(){
             toMotifPath = follower.pathBuilder()
                     .addPath(new BezierLine(startPose, motifDetectionPose))
                     .setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading())
                     .setHeadingConstraint(headingError)
                     .setTimeoutConstraint(3000)
                     .setTranslationalConstraint(pathDistThresholdMin)
                     .build();
             toShootingPath = follower.pathBuilder()
                     .addPath(new BezierLine(motifDetectionPose, shootPose))
                     .setLinearHeadingInterpolation(motifDetectionPose.getHeading(), shootPose.getHeading())
                     .setHeadingConstraint(headingError)
                     .setTranslationalConstraint(pathDistThresholdMin)
                     .setTimeoutConstraint(3000)
                     .build();
             leaveBasePath = follower.pathBuilder()
                     .addPath(new BezierLine(shootPose, leavePose))
                     .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                     .setHeadingConstraint(headingError)
                     .setTranslationalConstraint(pathDistThresholdMin)
                     .setTimeoutConstraint(3000)
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
                     firstPath,
                     motifCommand
             );

         }
         @Override
         protected void initializeMechanisms() {
             limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
             spindexer = new Spindexer(hardwareMap);
             shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);
         }

         @Override
         protected Command postMotifSequence(){
             limelight.stop();//temporarily turn it off to hand to localizer
             return new SequentialCommandGroup(
                     new WaitCommand(waitTime),
                     new FollowPathCommand(follower, toShootingPath, true),
//                new FacePose(follower, rightTargetPose),
                     new ShootHardcode(spindexer, shooter, motifPattern, true),
                     new FollowPathCommand(follower, leaveBasePath, true)
             );

         }

         protected void updateTelemetry(){
             follower.update();
             currentPose = follower.getPose();
             timer.getTime();
             addBooleanToTelem("First path is busy", firstPath.isFinished());
             addBooleanToTelem("Motif Busy", ConstantsBot.motifIsBusy);
             addStringToTelem("Motif timer", String.valueOf(motifCommand.timer.getTime()));
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




