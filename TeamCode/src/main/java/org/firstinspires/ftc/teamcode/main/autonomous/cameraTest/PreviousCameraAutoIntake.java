//package org.firstinspires.ftc.teamcode.main.autonomous.cameraTest;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.pedropathing.geometry.Pose;
//import com.seattlesolvers.solverslib.command.Command;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//
//import org.firstinspires.ftc.teamcode.commands.pathing.FollowPathCommand;
//
//import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand3;
//import org.firstinspires.ftc.teamcode.game.BallColor;
//import org.firstinspires.ftc.teamcode.game.ShootSide;
//import org.firstinspires.ftc.teamcode.main.autonomous.BaseAuto;
//import org.firstinspires.ftc.teamcode.commands.CamCommand;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
//import org.firstinspires.ftc.teamcode.util.ConfigNames;
//@Config
//@Configurable
//
//@Autonomous(name = "Prev Cam Auto Intake", group = "Test")
//public class PreviousCameraAutoIntake extends BaseAuto {
//    int limelightPipelineStart = 3;//CHANGE THIS
//    public static Pose startPose = new Pose(72, 8, Math.toRadians(0));//CHANGE THIS
//    CamCommand cam;
//    public static ShootSide shootSide = ShootSide.RIGHT;
//    @Override
//    protected Command preMotifSequence(){
//        cam = new CamCommand(limelight, follower, shootSide);
//        return cam;
//    }
//
//    public static double targetX = 130;
//    @Override
//    protected boolean isVisionComplete(){
//        return cam.isFinished();
//    }
//
//    public static boolean drive = true;
//    public static double inBetweenTime = 200;
//
//    @Override
//    protected Command postMotifSequence() {
//        //CamCommand cam = new CamCommand(limelight, follower);
//
//        if(drive){
//            return new ParallelCommandGroup(
//                    buildBallPathSequence(cam),
//                    new AutoIntakeCommand3(spindexer, intake, 0.8, inBetweenTime, true, hardwareMap)
//            );
//        }
//        return new InstantCommand();// new SequentialCommandGroup(
//                /*
//
//                //runs camera and movementing until it finds balls
//                        new RepeatCommand(cam),//does the camera thing
//                        new RepeatCommand(strafeLeftRight(8)),//moves around so you can detect balls
//                        new WaitUntilCommand(() -> !cam.finalBallList.isEmpty())
//
//*/
//        //when balls are found you go to them
////                buildBallPathSequence(cam);
//        //);
//    }
//    /*
//    @Override
//    protected Command postMotifSequence() {
//        CamCommand cam = new CamCommand(limelight, follower);
//
//        return new SequentialCommandGroup(
//                //runs camera and movementing until it finds balls
//                new ParallelRaceGroup(
//                        new RepeatCommand(cam),//does the camera thing
//                        new RepeatCommand(strafeLeftRight(8)),//moves around so you can detect balls
//                        new WaitUntilCommand(() -> !cam.finalBallList.isEmpty())
//                ),
//
//                //when balls are found you go to them
//                buildBallPathSequence(cam)
//        );
//    }
//    */
//    /*
//    @Override//this one is chatgpt
//    protected Command postMotifSequence() {
//        CamCommand cam = new CamCommand(limelight, follower);
//
//        return new SequentialCommandGroup(
//                new ParallelRaceGroup(
//                        new RepeatCommand(cam),
//                        new RepeatCommand(strafeLeftRight(8)),
//                        new WaitUntilCommand(() -> cam.finalBallList != null && !cam.finalBallList.isEmpty())
//                ),
//
//                new InstantCommand(() -> {
//                    new FollowPathCommand(
//                            follower,
//                            cam.getList(follower)
//                    ).schedule();
//                })
//        );
//    }
//*/
//
//    private Command buildBallPathSequence(CamCommand camera) {
//        /*
//        ParallelCommandGroup parallel = new ParallelCommandGroup();
//        //parallel.addCommands(new AutoIntakeCommand3(spindexer, intake, 0.5, 0.5, false, hardwareMap));
//        if(camera.finalBallList != null && camera.finalBallList.size() > 0) {
//            parallel.addCommands(new FollowPathCommand(follower, camera.getList(follower)));
//        }
//        */
//        if(camera.getList(follower, targetX)!= null && camera.getList(follower, targetX).getPath(0)!= null) {
//            return new FollowPathCommand(follower, camera.getList(follower, targetX).getPath(0));
//        } else{
//            return new InstantCommand();
//        }
//    }
//    private Command strafeLeftRight(double distance){
//        Pose botPose = follower.getPose();
//
//        double leftX = -1 * distance * Math.cos(Math.toRadians(90-botPose.getHeading()));//the amount left you have to go to go left
//        double leftY = distance * Math.sin(Math.toRadians(90-botPose.getHeading()));//amount up you go to go left
//        Pose leftPose = new Pose(leftX, leftY, botPose.getHeading());//gets pose from that
//
////        double rightX = distance * Math.cos(Math.toRadians(90-botPose.getHeading()));//amount right to go right
////        double rightY = -1 * distance * Math.sin(Math.toRadians(90-botPose.getHeading()));//amount down to go right
////        Pose rightPose = new Pose(rightX, rightY, botPose.getHeading());//gets pose from that
//
//        return new SequentialCommandGroup(
//                new FollowPathCommand(follower, leftPose, 0.5)//this moves left
////                new FollowPathCommand(follower, rightPose, 0.5)//this moves right
//        );
//    }
//
//    @Override
//    protected void initializeMechanisms() {
//        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);//init intak
//        spindexer = new Spindexer(hardwareMap, true).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}).initAngle();//init spindexer
//    }
//
//    @Override
//    protected void setupVision() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
//        limelight.pipelineSwitch(limelightPipelineStart);
//        limelight.start();
//    }
//
//    @Override
//    protected Pose getStartPose(){
//        return startPose;
//    }
//
//    @Override
//    protected void updateTelemetry(){
//        if(cam != null) {
//            telemetry.addData("camcommand finished", cam.isFinished());
//        }
//        telemetry.addData("Robot position", follower.getPose());
//        telemetry.update();
//        if(cam != null && cam.getList(follower, targetX) == null || cam.getList(follower, targetX).getPath(0)== null) {
//            telemetry.addData("first one is null", "null");
//        }
//        else{
////            telemetry.addData("start", cam.getList(follower).getPath(0).getPose(0));
////            telemetry.addData("final", cam.getList(follower).getPath(0).getPose(1));
////            telemetry.addData("closest", cam.getMinBallPose().getX());
////            telemetry.addData("closest", cam.getMinBallPose().getY());
//            for(Pose pose : cam.getFinalBallList()){
//                telemetry.addData("pose", pose);
//            }
//
//            telemetry.addLine("GLOBAL");
//            for(Pose pose : cam.getFinalGlobalPoseList()){
//                telemetry.addData("pose", pose);
//            }
//        }
//        follower.update();
//    }
//}
//
