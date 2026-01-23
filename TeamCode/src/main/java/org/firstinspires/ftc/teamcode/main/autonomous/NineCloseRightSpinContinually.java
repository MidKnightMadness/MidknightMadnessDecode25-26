//package org.firstinspires.ftc.teamcode.main.autonomous;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.seattlesolvers.solverslib.command.Command;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//
//
//import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.intake.IntakeTimeCommand;
//import org.firstinspires.ftc.teamcode.commands.readwrite.MotifWriteCommand;
//import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
//import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
//import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoSpot;
//import org.firstinspires.ftc.teamcode.game.BallColor;
//import org.firstinspires.ftc.teamcode.game.MotifEnums;
//import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
//import org.firstinspires.ftc.teamcode.game.SpotType;
//import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
//import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
//import org.firstinspires.ftc.teamcode.util.ConfigNames;
//import org.firstinspires.ftc.teamcode.game.ShootSide;
//
//@Config
//@Configurable
//@Disabled
//@Autonomous(name = "9 Close Right NonSorted SpinContinually", group = "Competition")
//public class NineCloseRightSpinContinually extends BaseAuto {
//    public static double motifDetectionTimeMs = 3000;
//    int startPipeline = 1;
//    public static Pose startPose = new Pose(118, 130, Math.toRadians(220));
//    //    public static Pose motifDetectionPose = new Pose(87, 94, Math.toRadians(100));
//    public static Pose shootPose = new Pose(87, 94, Math.toRadians(225));
//    public static Pose secondShootPose = new Pose(87, 94, Math.toRadians(221));
//    public static Pose parkPose = new Pose(85, 109, Math.toRadians(0));
//    public static Pose openGatePose = new Pose(136, 76, Math.toRadians(180));
//    public static Pose intakeOnePose = new Pose(102, 84, Math.toRadians(0));
//    public static Pose intakeTwoPose = new Pose(102, 60, Math.toRadians(0));
//    public static Pose intakeThreePose = new Pose(102, 36, Math.toRadians(0));
//    MotifEnums.Motif motifPattern = MotifEnums.Motif.GPP;
//    MotifWriteCommand motifCommand = null;
//
//    ShootSide shootSide = ShootSide.RIGHT;
//    Pose currentPose;
//
//    Command firstPath;
//    public static long firstWaitTime = 800;
//    public static long secondWaitTime = 200;
//    public static long thirdWaitTime = 200;//250 old
//
//    public static long fourthWaitTime = 700;
//
//    public static double pathDistThresholdMin = 1.5;
//    public static double headingError = Math.toRadians(2);
//    public static double timeOutConstraint = 200;
//    public static double tValueConstraint = 0.98;
//    public static double xChangeIntake = 23;
//    public static int[] shootArray = new int[]{2, 1, 0};
//
//    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.RawPower;
//
//    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
//    SpindexerSpot[] spots;
//
//    ShootSeqCommand seqShootCommand;
//    public static long moveGreenWaitTime = 150;
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
////        toMotifPath = follower.pathBuilder()
////                .addPath(new BezierLine(startPose, motifDetectionPose))
////                .setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading())
////                .setHeadingConstraint(headingError)
////                .setTimeoutConstraint(timeOutConstraint)
////                .setTranslationalConstraint(pathDistThresholdMin)
////                .setTValueConstraint(0.97)
////                .build();
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
////        if(motifCommand.getDetected() != MotifEnums.Motif.NONE){
////            motifPattern = motifCommand.getDetected();
////        }
////        if(motifCommand.isFinished()){
////            return true;
////        }
//        return true;
//    }
//    public static long waitTime = 500;
//    AutoIntakeCommand autoIntakeCommand;
//    boolean autoStart = false;
//
//    @Override
//    protected Command preMotifSequence(){
////        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
////
////        firstPath = new FollowPathCommand(follower, , true).setGlobalMaxPower(0.9);
////        return new SequentialCommandGroup(
////                setDefaultStartColors(),
////                firstPath,
////                motifCommand
////        );
//        return null;
//
//    }
//    @Override
//    public void update(){
//
//    }
//    @Override
//    protected void initializeMechanisms() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
//        spindexer = new Spindexer(hardwareMap, false, false).setBallColors(startBallColors).initAngle();
//
//        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
////        shooter.setRunMode(TwoWheelShooter.RunMode.RawPower);
//        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
//
//
//        if(shooterRunMode == TwoWheelShooter.RunMode.VelocityControl) {
//            shooter.low.setVeloCoefficients(pidBotGainsShooter[0], pidBotGainsShooter[1], pidBotGainsShooter[2]);
//            shooter.high.setVeloCoefficients(pidTopGainsShooter[0], pidTopGainsShooter[1], pidTopGainsShooter[2]);
//            shooter.low.setFeedforwardCoefficients(kBotGainsShooter[0], kBotGainsShooter[1], kBotGainsShooter[2]);
//            shooter.high.setFeedforwardCoefficients(kTopGainsShooter[0], kTopGainsShooter[1], kTopGainsShooter[2]);
//        }
//    }
//
//
//    //    @Override
////    public Command goToIntakeLine(){
////        return new SequentialCommandGroup(
////
////        );
////    }
//    @Override
//    protected Command postMotifSequence(){
//        limelight.stop();
//        //temporarily turn it off to hand to localizer
//        return new SequentialCommandGroup(
////                getToShootCommand(1, 1000),
////                new ParallelDeadlineGroup(
////                        new FlywheelShootTimed(shooter, follower, shootSide,  TwoWheelShooter.ShootDist.Close, false, 5000, false)
////                ),
//                new ParallelCommandGroup(
//                        new InstantCommand(()-> shooter.setFlywheelStaticPresets(TwoWheelShooter.ShootDist.Close, true)),
//                        new SequentialCommandGroup(
//                                getToShootCommand(1, 500),
//                                new WaitCommand(500),
//                                new InstantCommand(()-> spindexer.spin(-0.25))
//                        )
//                ),
//                new WaitCommand(2000),
//                new InstantCommand(()-> shooter.stopFlywheels()),
//                //new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
//                //new WaitCommand(1000),
//                new InstantCommand(() -> spindexer.getTurner().getServo().setPower(0)),
//                new ParallelCommandGroup(
//                        getToLineNum(1, 200),
////                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
//                        new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0)
//                ),
//                new WaitCommand(200),
//                intake(1, 0),
//                new InstantCommand(() -> spindexer.getTurner().getServo().setPower(0)),
//                setDefaultStartColors(),
////                new ParallelCommandGroup(
////                getToShootCommand(2, 0),
//                new WaitCommand(1000),
//                new ParallelCommandGroup(
//                        new InstantCommand(()-> shooter.setFlywheelStaticPresets(TwoWheelShooter.ShootDist.Close, true)),
//                        new SequentialCommandGroup(
//                                getToShootCommand(2, 500),
//                                new WaitCommand(500),
//                                new InstantCommand(()-> spindexer.spin(-0.25))
//                        )
//                ),
//                new WaitCommand(2000),
//                new InstantCommand(()-> shooter.stopFlywheels()),
//                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
//                // new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT1, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
//                new ParallelCommandGroup(
//                        getToLineNum(2, 200),
////                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
//                        new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0)
//                ),
////                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
//                //new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
//                new WaitCommand(200),
//                intake(2, 0),
//
//                new InstantCommand(() -> spindexer.getTurner().getServo().setPower(0)),
//                setDefaultStartColors(),
////                new ParallelCommandGroup(
////                getToShootCommand(2, 0),
//                new WaitCommand(1000),
//                new ParallelCommandGroup(
//                        new InstantCommand(()-> shooter.setFlywheelStaticPresets(TwoWheelShooter.ShootDist.Close, true)),
//                        new SequentialCommandGroup(
//                                getToShootCommand(2, 500),
//                                new WaitCommand(500),
//                                new InstantCommand(()-> spindexer.spin(-0.25))
//                        )
//                ),
//                new WaitCommand(2000),
//                new InstantCommand(()-> shooter.stopFlywheels()),
//                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
//                //new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 1000),
////                openGate(1000),
//////
////                line2Commands(),
////                line3Commands(),
//
//                park(100)
////
////               new InstantCommand(() -> spindexer.goTo(Angle.fromDegrees(0), CRServoEx2.RunMode.OptimizedPositionalControl))
////
//        );
////        else{
////        return null;
//
//    }
//
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
//    protected SequentialCommandGroup openGate(long milliSec){
//        return new SequentialCommandGroup(
//                new SchedulePathTo(follower, openGatePose, headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint),
//                new WaitCommand(milliSec)
//        );
//    }
//    protected Command intake(int targetSpot, int initialSpindexerIntakeSpot){
////        autoIntakeCommand = new AutoIntakeCommand(spindexer, intake, intakePower, intakeTime);
//        return new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> intake.setDirectPower(1.0)),
//                        new InstantCommand(()-> spindexer.spin(-0.3)),
//                        driveToIntakeEnd(targetSpot)
//                ).withTimeout(3500),
//                new InstantCommand(()-> spindexer.setDefaultAngleTolerance()),
//                new InstantCommand(()-> spindexer.getTurner().setPIDFTOUse(spindexer.outtakeTurnerCoeff)),
//                new InstantCommand(()-> intake.stopPower())
//        );
//
////        return intakePower(milliSec);
//    }
//
//    protected SequentialCommandGroup driveToIntakeEnd(int spot){
//        Pose intakePose = (spot == 1) ? intakeOnePose : (spot == 2) ? intakeTwoPose : intakeThreePose;
//
////        follower.update();
//        if(spot != 2) {
//            return new SequentialCommandGroup(
//                    new SchedulePathTo(follower, new Pose(intakePose.getX() + xChangeIntake, intakePose.getY(), intakePose.getHeading()), headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint)
//                            .setMaxPower(0.3)
//            );
//        }
//        else{
//            return new SequentialCommandGroup(
//                    new SchedulePathTo(follower, new Pose(intakePose.getX() + xChangeIntake + 6, intakePose.getY(), intakePose.getHeading()), headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint)
//                            .setMaxPower(0.3)
//            );
//        }
//    }
//
//    protected SequentialCommandGroup park(long milliSec){
//        return new SequentialCommandGroup(
//                new SchedulePathTo(follower, parkPose, headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint),
//                new WaitCommand(milliSec)
//        );
//    }
//    protected SequentialCommandGroup startShoot(){
//        return new SequentialCommandGroup(
//                new ShootSeqCommand(spindexer, shooter, SpindexerSpot.convertFromindex(shootArray), follower, shootSide, false, TwoWheelShooter.ShootDist.Close, true)
//        );
//    }
//    protected SequentialCommandGroup shootOptimal(MotifEnums.Motif pattern){
//        BallColor[] colors = new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
//        BallColor[] PPG = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
//        BallColor[] PGP = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
//
//        if (pattern == MotifEnums.Motif.GPP) {
////            if (Arrays.equals(colors, PPG)) {
//            spots = SpindexerSpot.convertFromindex(new int[]{2, 1, 0});
////            } else if (Arrays.equals(colors, PGP)) {
////                spots = SpindexerSpot.convertFromindex(new int[]{1, 2, 0});
////            }
//            //    }
//        }
//        else if (pattern == MotifEnums.Motif.PGP) {
////            if (Arrays.equals(colors, PPG)) {
//            spots = SpindexerSpot.convertFromindex(new int[]{1, 2, 0});
////            } else if (Arrays.equals(colors, PGP)) {
////                spots = SpindexerSpot.convertFromindex(new int[]{2, 1, 0});
////            }
//        }
//        else if (pattern == MotifEnums.Motif.PPG) {
//            // if (Arrays.equals(colors, PPG)) {
//            spots = SpindexerSpot.convertFromindex(new int[]{1, 0, 2});
////            } else if (Arrays.equals(colors, PGP)) {
////                spots = SpindexerSpot.convertFromindex(new int[]{2, 0, 1});
////            }
//        }
//        else {
//            spots = SpindexerSpot.convertFromindex(new int[]{1, 0, 2});
//        }
//
//        seqShootCommand = new ShootSeqCommand(spindexer, shooter, spots, follower, shootSide, false, TwoWheelShooter.ShootDist.Close, true);
//        return new SequentialCommandGroup(
//                seqShootCommand
////                new InstantCommand(() -> shooter.setFlywheelsPower(TwoWheelShooter.ShootDist.Close)),
////                new SpindexerGotoSpot(spindexer, spots[0], SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 200),
////                new WaitCommand(2000),
////                new SpindexerGotoSpot(spindexer, spots[1], SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 200),
////                new WaitCommand(2000),
////                new SpindexerGotoSpot(spindexer, spots[2], SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 200),
////                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}))
//        );
//    }
//    protected SequentialCommandGroup getToLineNum(int lineNum, long milliSec){
//        SchedulePathTo command = null;
//        if(lineNum == 1) command = new SchedulePathTo(follower, intakeOnePose, headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint);
//        else if(lineNum == 2) command =  new SchedulePathTo(follower, intakeTwoPose, headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint);
//        else command = new SchedulePathTo(follower, intakeThreePose, headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint);
//
//        return new SequentialCommandGroup(
//                command
//        );
//    }
//
//    protected SequentialCommandGroup getToShootCommand(int num, long millSec){
//        if(num == 1) {
//            return new SequentialCommandGroup(
//                    new SchedulePathTo(follower, shootPose, headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint).setMaxPower(1.0),
//                    new WaitCommand(millSec)
//            );
//        }
//        else{
//            return new SequentialCommandGroup(
//                    new SchedulePathTo(follower, secondShootPose, headingError, timeOutConstraint, pathDistThresholdMin, tValueConstraint).setMaxPower(1.0),
//                    new WaitCommand(millSec)
//            );
//        }
//    }
//
//
//    protected void updateTelemetry(){
//        // Update pose & follower
//        follower.update();
//        currentPose = follower.getPose();
//        double currentTime = gameTimer.getTime();
//
//        // Follower
//        telemetry.addData("Current Voltage", shooter.getCurrVoltage());
//        telemetry.addData("Ratio Voltage ", shooter.getTargetVoltage() / shooter.getCurrVoltage());
//        telemetry.addData("Current Follower Pose", currentPose.getPose());
//        telemetry.addData("Follower Velocity", follower.getVelocity());
//        telemetry.addData("Start Ball Color 0", startBallColors[0]);
//        telemetry.addData("Start Ball Color 1", startBallColors[1]);
//        telemetry.addData("Start Ball Color 2", startBallColors[2]);
//
//        telemetry.addData("New Ball Detected", spindexer.newBallDetected());
//        if(seqShootCommand != null) {
//            telemetry.addData("Seq Test Farthest Moved", seqShootCommand.farthestMoved);
//        }
//
//        telemetry.addData("Motif", motifPattern);
//        if(spindexer.getBallColors() != null) {
//            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
//            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
//            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
//        }
//        if(spots != null) {
//            telemetry.addData("Spindexer Optimal Sequence 0", spots[0]);
//            telemetry.addData("Spindexer Optimal Sequence 1", spots[1]);
//            telemetry.addData("Spindexer Optimal Sequence 2", spots[2]);
//        }
//        telemetry.addData("All Occupied", spindexer.allOccuppiedBallColors());
//
//        addToTelemGraph("Pose X", currentPose.getX());
//        addToTelemGraph("Pose Y", currentPose.getY());
//        addToTelemGraph("Pose Heading", currentPose.getHeading());
//        addToTelemGraph("Follower T Value", follower.getCurrentTValue());
//        addToTelemGraph("Follower Velocity X", follower.getVelocity().getXComponent());
//        addToTelemGraph("Follower Velocity Y", follower.getVelocity().getYComponent());
//        addToTelemGraph("Follower Velocity Mag", follower.getVelocity().getMagnitude());
//        addToAllTelemGraph("Follower Velocity Heading", follower.getVelocity().getTheta());
//        addToAllTelemGraph("Follower Translational Error", follower.getDriveError());
//        addToAllTelemGraph("Follower Heading Error", follower.getHeadingError());
//        addToTelemGraph("Follower Max Vel Constraint", follower.getConstraints().getVelocityConstraint());
//        addToTelemGraph("Follower T Constraint", follower.getConstraints().getTValueConstraint());
//
//        //Shooter
//        if(shooter != null){
//            addToAllTelemGraph("Shooter Low Flywheel Power", shooter.low.get());
//            addToAllTelemGraph("Shooter High Flywheel Power", shooter.high.get());
//            addToAllTelemGraph("Shooter Low Flywheel Vel", shooter.getPredictedBotVel());
//            addToAllTelemGraph("Shooter High Flywheel Vel", shooter.getPredictedTopVel());
//            telemetry.addData("Shooter Dir RunMode", shooter.runMode);
//            telemetry.addData("Shooter RunMode", shooterRunMode);
//            telemetry.addData("Shooter Low RunMode", shooter.low.motorEx.getMode());
//            telemetry.addData("Shooter High RunMode", shooter.low.motorEx.getMode());
//        }
//
//        //Spindexer
//        if(spindexer != null){
//            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
//            telemetry.addData("Balls Left", spindexer.getBallCount());
//            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
//        }
//
//        //Intake
//        if(intake != null){
//            addToAllTelemGraph("Intake Power", intake.getMotor().get());
//            addToAllTelemGraph("Intake Velocity", intake.getMotorVelocity());
//        }
//
//        //Time
//        addToAllTelemGraph("Auto Elapsed Time", currentTime);
//        addToAllTelemGraph("Update Rate", 1 / gameTimer.getDeltaTime());
//
//        //Motif
//        addBooleanToTelem("Motif Busy", !isVisionComplete());
//        if(motifCommand != null){
//            addStringToTelem("Motif Timer", String.valueOf(motifCommand.getTime()));
//        }
//        addStringToTelem("Motif Pattern", String.valueOf(motifPattern));
//
//        //First Path
//        if(firstPath != null){
//            addBooleanToTelem("First Path Busy", !firstPath.isFinished());
//        }
//        if(seqShootCommand != null) {
//            telemetry.addData("Spindexer Shoot CurrBallIndex", seqShootCommand.getCurrBallIndex());
//        }
//        telemetry.addData("Spindexer Get Curr Angle", spindexer.getCurrentAngle());
////
//
//        telemetry.update();
//        graphManager.update();
//        telemetryManager.update();
//        if(dashboard!= null) {
//            dashboard.sendTelemetryPacket(dashboardPacket);
//        }
//    }
//
//
//
//    public void addStringToTelem(String s, String o){
//        telemetry.addLine(s + o);
//    }
//    public void addToTelemGraph(String s, Number o){
//        telemetryManager.addData(s, o);
//        graphManager.addData(s, o);
//    }
//    public void addToAllTelemGraph(String s, Number o){
//        telemetryManager.addData(s, o);
//        graphManager.addData(s, o);
//        telemetry.addData(s, o);
//        if(dashboard != null) {
//            dashboardPacket.put(s, o);
//        };
//    }
//    public void addBooleanToTelem(String s, boolean o){
//        telemetry.addData(s, o);
//        telemetryManager.addData(s, o);
//    }
//
//
//
//
//}
