
 package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.game.ShootSide;

 @Config
     @Configurable
     @Autonomous(name = "Back Right", group = "Competition")
     public class ThreeBallBackRightAuto extends BaseAuto {
//         public static double motifDetectionTimeMs = 5000;
         int startPipeline = 1;

         public static Pose startPose = new Pose(88, 8, Math.toRadians(90));
         //    public static Pose startToShootControlPose = new Pose(54, 19);
         public static Pose shootPose = new Pose(84, 17, Math.toRadians(68));
         public static Pose leavePose = new Pose(86, 38, Math.toRadians(68));
         PathChain toShootingPath;
         PathChain leaveBasePath;
//         MotifEnums.Motif motifPattern;
//         MotifWriteCommand motifCommand;

         ShootSide shootSide = ShootSide.RIGHT;
         Pose currentPose;

         double speed;
         double acc;

         public static long waitTime = 5000;
         public static double pathDistThresholdMax = 3;
         public static double headingErrorMax = 0.3;

         @Override
         protected Pose getStartPose(){
             return startPose;
         }

//         @Override
//         protected void setupVision(){
//             limelight.pipelineSwitch(startPipeline);
//             limelight.start();
//         }

         @Override
         protected void buildPaths(){
             toShootingPath = follower.pathBuilder()
                     .addPath(new BezierLine(startPose, shootPose))
                     .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                     .setHeadingConstraint(headingErrorMax)
                     .setTimeoutConstraint(3000)
                     .setTranslationalConstraint(pathDistThresholdMax)
                     .build();
             leaveBasePath = follower.pathBuilder()
                     .addPath(new BezierLine(shootPose, leavePose))
                     .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                     .setHeadingConstraint(headingErrorMax)
                     .setTimeoutConstraint(3000)
                     .setTranslationalConstraint(pathDistThresholdMax)
                     .build();
         }


//         @Override
//         protected boolean isVisionComplete(){
//             motifPattern = motifCommand.getDetected();
//             if(motifPattern != MotifEnums.Motif.NONE){
//                 ConstantsBot.motifIsBusy = false;
//                 return true;
//             }
//             ConstantsBot.motifIsBusy = true;
//             return false;
//         }

//         @Override
//         protected Command preMotifSequence(){
//             motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
//             return new SequentialCommandGroup(
//                     motifCommand
//             );
//
//         }
@Override
protected void initializeMechanisms() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
    spindexer = new Spindexer(hardwareMap);
    shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);
}

     @Override
         protected Command postMotifSequence(){
             return new SequentialCommandGroup(
                     new FollowPathCommand(follower, toShootingPath, true).setGlobalMaxPower(0.6),
                     new WaitCommand(waitTime),
//                     new FacePose(follower, rightTargetPose),
//                     new WaitCommand(waitTime),
//                     new ShootHardcode(spindexer, shooter, motifPattern, false)
                     new WaitCommand(waitTime),
                     new FollowPathCommand(follower, leaveBasePath, true)
             );

         }

         protected void updateTelemetry(){
             follower.update();
             currentPose = follower.getPose();
             timer.getTime();
//             addStringToTelem("Motif Pattern", String.valueOf(motifPattern));
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

