//package org.firstinspires.ftc.teamcode.old.opModes;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.seattlesolvers.solverslib.command.Command;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
//
//
//import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.intake.IntakeTimeCommand;
//import org.firstinspires.ftc.teamcode.commands.readWrite.MotifWriteCommand;
//import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
//
//import org.firstinspires.ftc.teamcode.commands.shooter.ShootSeqCommand;
//import org.firstinspires.ftc.teamcode.game.BallColor;
//import org.firstinspires.ftc.teamcode.game.MotifEnums;
//import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
//import org.firstinspires.ftc.teamcode.main.autonomous.BaseAuto;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
//import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
//import org.firstinspires.ftc.teamcode.util.ConfigNames;
//import org.firstinspires.ftc.teamcode.game.ShootSide;
//
//import java.util.Arrays;
//
//@Config
//@Configurable
//@Disabled
//@Autonomous(name = "3 Back Left", group = "Competition")
//public class ThreeBackLeftAuto extends BaseAuto {
//    public static double motifDetectionTimeMs = 3000;
//    int startPipeline = 1;
//    public static Pose startPose = new Pose(144-88, 8,  Math.toRadians(90));
//    public static Pose driveForwardPose = new Pose(144- 87, 14, Math.toRadians(90));
//    public static Pose shootPose = new Pose(144- 84, 17, Math.toRadians(285));
//    public static Pose leavePose = new Pose(8, 8, Math.toRadians(0));
//
//
//    PathChain toMotifPath;
//    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
//    MotifWriteCommand motifCommand = null;
//
//    public static ShootSide shootSide = ShootSide.LEFT;
//    Pose currentPose;
//
//    Command firstPath;
//
//    public static double pathDistThresholdMin = 0.5;
//    public static double headingError = 0.025;
//    public static double timeOutConstraint = 200;
//    public static double tValueConstraint = 0.97;
//    public static double intakeTime = 6000;
//    public static double intakePower = 1;
//    public static double xChangeIntake = 20;
//    public static int[] shootArray = new int[]{2, 1, 0};
//
//    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;
//
//    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
//    SpindexerSpot[] seq = SpindexerSpot.convertFromindex(new int[]{0, 0, 0});
//    long intakeHelpTime = 3000;
//    SpindexerSpot[] spots;
//
//    ShootSeqCommand seqTestCommand;
//    PathChain forwardPath;
//    @Override
//    protected Pose getStartPose(){
//        return startPose;
//    }
//
//    @Override
//    protected void setupVision(){
//        limelight.pipelineSwitch(startPipeline);
//        limelight.start();
//    }
//
//    @Override
//    protected ShootSide getSide(){
//        return shootSide;
//    }
//
//    //keep these empty and build the path using follower's current Pose
//    @Override
//    protected void buildPaths(){
//        forwardPath = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, driveForwardPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), driveForwardPose.getHeading())
//                .setHeadingConstraint(headingError)
//                .setTimeoutConstraint(timeOutConstraint)
//                .setTranslationalConstraint(pathDistThresholdMin)
//                .setTValueConstraint(tValueConstraint)
//                .build();
//    }
//
//
////    @Override
////    public BallColor[] getStartBallColors(){
////        return startBallColors;
////    }
//
//    @Override
//    protected boolean isVisionComplete(){
//        if(motifCommand.getDetected() == MotifEnums.Motif.NONE){
//            motifPattern = motifCommand.getDetected();
//        }
//        if(motifCommand.isFinished()){
//            return true;
//        }
//        return false;
//    }
//    public static long waitTime = 1000;
//    AutoIntakeCommand autoIntakeCommand;
//    boolean autoStart = false;
//
//    @Override
//    protected Command preMotifSequence(){
//        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
//
////        firstPath = new FollowPathCommand(follower, toMotifPath, true).setGlobalMaxPower(0.9);
//        return new SequentialCommandGroup(
//                setDefaultStartColors(),
//                forwardCommand(500),
//                motifCommand
////                firstPath,
//        );
//
//    }
//    @Override
//    public void update(){
//
//    }
//    @Override
//    protected void initializeMechanisms() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
//        spindexer = new Spindexer(hardwareMap, false).setBallColors(startBallColors);
//
//        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
////        shooter.setRunMode(TwoWheelShooter.RunMode.RawPower);
//        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
//
//
//    }
//
//    @Override
//    protected Command postMotifSequence(){
//        limelight.stop();//temporarily turn it off to hand to localizer
//        return new SequentialCommandGroup(
////              getToShootCommand(1000)
//                new WaitCommand(waitTime),
//                getToShootCommand(1500),
////                startShoot()
//                shootOptimal(motifPattern),
////                intakePower(intakeHelpTime)),
////
////                getToLineNum(1, 500),
////                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 1000),
////                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
////                intake(1, 5000),
////                openGate(1000),
////
////                getToShootCommand(500)
////              shoot(),
////
//                //    getToLineNum(2, 500),
////              intake(1),
//                //   getToShootCommand(500),
////              shoot(),
////
//                //  getToLineNum(3, 500),
////              intake(3),
//                //  getToShootCommand(500),
////              shoot(),
////
//                park(100)
////                new InstantCommand(() -> spindexer.goTo(Angle.fromDegrees(0), CRServoEx2.RunMode.OptimizedPositionalControl))
////
//        );
////        else{
////        return null;
//
//    }
//
//    private SequentialCommandGroup forwardCommand(long waitTime) {
//        return new SequentialCommandGroup(
//                new FollowPathCommand(follower, forwardPath).setGlobalMaxPower(0.7),
//                new WaitCommand(waitTime)
//        );
//    }
//
//    private Command intakePower(long milliSec) {
//        return new IntakeTimeCommand(intake, milliSec);
//    }
//
//    //    protected SequentialCommandGroup goToMotifDetection(long milliSec){
////        return new SequentialCommandGroup(
////                new SchedulePathTo(follower, motifDetectionPose, headingError, timeOutConstraint, pathDistThresholdMin),
////                new WaitCommand(milliSec)
////        );
////    }
//    protected SequentialCommandGroup setDefaultStartColors(){
//        return new SequentialCommandGroup(
//                new InstantCommand(() -> spindexer.setBallColors(startBallColors))
//        );
//    }
////    protected SequentialCommandGroup openGate(long milliSec){
////        return new SequentialCommandGroup(
////                new SchedulePathTo(follower, openGatePose, headingError, timeOutConstraint, pathDistThresholdMin),
////                new WaitCommand(milliSec)
////        );
////    }
//
//    protected Command intake(int spot, long milliSec){
////        autoIntakeCommand = new AutoIntakeCommand(spindexer, intake, intakePower, intakeTime);
////        autoStart = true;
////        return new ParallelCommandGroup(
////                autoIntakeCommand
//////                driveToIntakeEnd(spot, milliSec)
////        );
//        return intakePower(milliSec);
//    }
////    protected SequentialCommandGroup driveToIntakeEnd(int spot, long milliSec){
////        Pose intakePose = (spot == 1) ? intakeOnePose : (spot == 2) ? intakeTwoPose : intakeThreePose;
////
////        follower.update();
////        return new SequentialCommandGroup(
////                new SchedulePathTo(follower, new Pose(intakePose.getX() + xChangeIntake, intakePose.getY(), intakePose.getHeading()), headingError, timeOutConstraint, pathDistThresholdMin),
////                new WaitCommand(milliSec));
////    }
//
//    protected SequentialCommandGroup park(long milliSec){
//        return new SequentialCommandGroup(
//                new SchedulePathTo(follower, leavePose, headingError, timeOutConstraint, pathDistThresholdMin),
//                new WaitCommand(milliSec)
//        );
//    }
//    protected SequentialCommandGroup startShoot(){
//        return new SequentialCommandGroup(
//                new ShootSeqCommand(spindexer, shooter, SpindexerSpot.convertFromindex(shootArray), follower, shootSide, false, TwoWheelShooter.ShootDist.Far, true)
//        );
//    }
//    protected SequentialCommandGroup shootOptimal(MotifEnums.Motif pattern){
//        if(pattern == MotifEnums.Motif.GPP){
//            if(Arrays.equals(spindexer.getBallColors(), new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN})){
//                spots = SpindexerSpot.convertFromindex(new int[]{2, 1, 0});
//            }
//            else if(Arrays.equals(spindexer.getBallColors(), new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE})){
//                spots = SpindexerSpot.convertFromindex(new int[]{1, 2, 0});
//            }
//        }else if(pattern == MotifEnums.Motif.PGP){
//            if(Arrays.equals(spindexer.getBallColors(), new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN})){
//                spots = SpindexerSpot.convertFromindex(new int[]{1, 2, 0});
//            }
//            else if(Arrays.equals(spindexer.getBallColors(), new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE})){
//                spots = SpindexerSpot.convertFromindex(new int[]{2, 1, 0});
//            }
//        }else if(pattern == MotifEnums.Motif.PPG){
//            if (Arrays.equals(spindexer.getBallColors(), new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN})) {
//                spots = SpindexerSpot.convertFromindex(new int[]{1, 0, 2});
//            }
//            else if(Arrays.equals(spindexer.getBallColors(), new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE})){
//                spots = SpindexerSpot.convertFromindex(new int[]{2, 0, 1});
//            }
//
//        }else {
//            spots = SpindexerSpot.convertFromindex(new int[]{1, 0, 2});
//        }
//
//        seqTestCommand = new ShootSeqCommand(spindexer, shooter, spots, follower, shootSide, false, TwoWheelShooter.ShootDist.Far, true);
//        return new SequentialCommandGroup(
//                seqTestCommand,
//                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}))
//        );
//    }
//    protected SequentialCommandGroup getToLineNum(int lineNum, long milliSec){
//        SchedulePathTo command = null;
////        if(lineNum == 1) command = new SchedulePathTo(follower, intakeOnePose, headingError, timeOutConstraint, pathDistThresholdMin);
////        else if(lineNum == 2) command =  new SchedulePathTo(follower, intakeTwoPose, headingError, timeOutConstraint, pathDistThresholdMin);
////        else command = new SchedulePathTo(follower, intakeThreePose, headingError, timeOutConstraint, pathDistThresholdMin);
//
//        return new SequentialCommandGroup(
//                command,
//                new WaitCommand(milliSec)
//        );
//    }
//
//    protected SequentialCommandGroup getToShootCommand(long millSec){
//        return new SequentialCommandGroup(
//                new SchedulePathTo(follower, shootPose, headingError, timeOutConstraint, pathDistThresholdMin),
//                new WaitCommand(millSec)
//        );
//    }
//
////    @Override
////    protected void updateTelemetry(){
////        // Update pose & follower
////        follower.update();
////        currentPose = follower.getPose();
////        double currentTime = gameTimer.getTime();
////
////        // Follower
////        telemetry.addData("Current Voltage", shooter.getCurrVoltage());
////        telemetry.addData("Ratio Voltage ", shooter.getTargetVoltage() / shooter.getCurrVoltage());
////
////        telemetry.addData("Current Follower Pose", currentPose.getPose());
////        telemetry.addData("Follower Velocity", follower.getVelocity());
////        telemetry.addData("Start Ball Color 0", startBallColors[0]);
////        telemetry.addData("Start Ball Color 1", startBallColors[1]);
////        telemetry.addData("Start Ball Color 2", startBallColors[2]);
////
////        telemetry.addData("New Ball Detected", spindexer.newBallDetected());
////        if(seqTestCommand != null) {
////            telemetry.addData("Seq Test Farthest Moved", seqTestCommand.farthestMoved);
////        }
////
////        telemetry.addData("Motif", motifPattern);
////        if(spindexer.getBallColors() != null) {
////            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
////            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
////            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
////        }
////        if(spots != null) {
////            telemetry.addData("Spindexer Optimal Sequence 0", spots[0]);
////            telemetry.addData("Spindexer Optimal Sequence 1", spots[1]);
////            telemetry.addData("Spindexer Optimal Sequence 1", spots[2]);
////        }
////        telemetry.addData("All Occupied", spindexer.allOccuppiedBallColors());
////
////        addToTelemGraph("Pose X", currentPose.getX());
////        addToTelemGraph("Pose Y", currentPose.getY());
////        addToTelemGraph("Pose Heading", currentPose.getHeading());
////        addToTelemGraph("Follower T Value", follower.getCurrentTValue());
////        addToTelemGraph("Follower Velocity X", follower.getVelocity().getXComponent());
////        addToTelemGraph("Follower Velocity Y", follower.getVelocity().getYComponent());
////        addToTelemGraph("Follower Velocity Mag", follower.getVelocity().getMagnitude());
////        addToAllTelemGraph("Follower Velocity Heading", follower.getVelocity().getTheta());
////        addToAllTelemGraph("Follower Translational Error", follower.getDriveError());
////        addToAllTelemGraph("Follower Heading Error", follower.getHeadingError());
////        addToTelemGraph("Follower Max Vel Constraint", follower.getConstraints().getVelocityConstraint());
////        addToTelemGraph("Follower T Constraint", follower.getConstraints().getTValueConstraint());
////
////        //Shooter
////        if(shooter != null){
////            addToAllTelemGraph("Shooter Low Flywheel Power", shooter.low.get());
////            addToAllTelemGraph("Shooter High Flywheel Power", shooter.high.get());
////            addToAllTelemGraph("Shooter Low Flywheel Vel", shooter.low.getVelocity());
////            addToAllTelemGraph("Shooter High Flywheel Vel", shooter.high.getVelocity());
////            telemetry.addData("Shooter RunMode", shooterRunMode);
////            telemetry.addData("Shooter Low RunMode", shooter.low.motorEx.getMode());
////            telemetry.addData("Shooter High RunMode", shooter.low.motorEx.getMode());
////        }
////
////        //Spindexer
////        if(spindexer != null){
////            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
////            telemetry.addData("Balls Left", spindexer.getBallCount());
////            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
////        }
//
//        //Intake
////        if(intake != null){
////            addToAllTelemGraph("Intake Power", intake.getMotor().get());
////            addToAllTelemGraph("Intake Velocity", intake.getMotorVelocity());
////        }
////
////        //Time
////        addToAllTelemGraph("Auto Elapsed Time", currentTime);
////        addToAllTelemGraph("Update Rate", 1 / gameTimer.getDeltaTime());
////
////        //Motif
////        addBooleanToTelem("Motif Busy", !isVisionComplete());
////        if(motifCommand != null){
////            addStringToTelem("Motif Timer", String.valueOf(motifCommand.getTime()));
////        }
////        addStringToTelem("Motif Pattern", String.valueOf(motifPattern));
////
////        //First Path
////        if(firstPath != null){
////            addBooleanToTelem("First Path Busy", !firstPath.isFinished());
////        }
//
////        telemetry.update();
////
////    }
//
//
//
//
//
//
//}
//
