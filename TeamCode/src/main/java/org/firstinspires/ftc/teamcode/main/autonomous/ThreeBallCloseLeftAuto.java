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
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;


import org.firstinspires.ftc.teamcode.commands.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeTimeCommand;
import org.firstinspires.ftc.teamcode.commands.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.commands.TurnToCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.lang.reflect.WildcardType;
import java.util.Arrays;

@Config
@Configurable
@Autonomous(name = "3 Close Left", group = "Competition")
public class ThreeBallCloseLeftAuto extends BaseAuto {
    public static double motifDetectionTimeMs = 3000;
    int startPipeline = 1;
    public static Pose  startPose = new Pose(144 - 118, 130, Math.toRadians(135));
    public static Pose  motifDetectionPose = new Pose(144 -87, 94, Math.toRadians(80));
    public static Pose  shootPose = new Pose(144 - 87, 94, Math.toRadians(310));
    public static Pose parkPose = new Pose(144 - 114, 94, Math.toRadians(330));

    public static Pose openGatePose = new Pose(136, 76, Math.toRadians(180));
    public static Pose intakeOnePose = new Pose(110, 84, Math.toRadians(0));
    public static Pose intakeTwoPose = new Pose(110, 60, Math.toRadians(0));
    public static Pose intakeThreePose = new Pose(110, 36, Math.toRadians(0));

    public static double intakeDistForward = 14;
    PathChain toMotifPath;
    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    MotifWriteCommand motifCommand = null;

    ShootSide shootSide = ShootSide.LEFT;
    Pose currentPose;

    Command firstPath;

    public static double pathDistThresholdMin = 0.5;
    public static double headingError = 0.025;
    public static double timeOutConstraint = 200;
    public static double xChangeIntake = 15;
    public static int[] shootArray = new int[]{2, 1, 0};

    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;

    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    SpindexerSpot[] spots;

    ShootSeqCommand seqShootCommand;
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

    //keep these empty and build the path using follower's current Pose
    @Override
    protected void buildPaths(){
        toMotifPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, motifDetectionPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading())
                .setHeadingConstraint(headingError)
                .setTimeoutConstraint(timeOutConstraint)
                .setTranslationalConstraint(pathDistThresholdMin)
                .setTValueConstraint(0.97)
                .build();
    }


//    @Override
//    public BallColor[] getStartBallColors(){
//        return startBallColors;
//    }

    @Override
    protected boolean isVisionComplete(){
        if(motifCommand.getDetected() == MotifEnums.Motif.NONE){
            motifPattern = motifCommand.getDetected();
            return true;
        }
        return motifCommand.isFinished();
    }
    public static long waitTime = 500;
    AutoIntakeCommand autoIntakeCommand;
    boolean autoStart = false;

    @Override
    protected Command preMotifSequence(){
        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);

        firstPath = new FollowPathCommand(follower, toMotifPath, true).setGlobalMaxPower(0.7);
        return new SequentialCommandGroup(
                setDefaultStartColors(),
                firstPath,
                motifCommand
        );

    }
    @Override
    public void update(){

    }
    @Override
    protected void initializeMechanisms() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        spindexer = new Spindexer(hardwareMap, false).setBallColors(startBallColors).initAngle();

        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
//        shooter.setRunMode(TwoWheelShooter.RunMode.RawPower);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);

        if(shooterRunMode == TwoWheelShooter.RunMode.VelocityControl) {
            shooter.low.setVeloCoefficients(pidBotGainsShooter[0], pidBotGainsShooter[1], pidBotGainsShooter[2]);
            shooter.high.setVeloCoefficients(pidTopGainsShooter[0], pidTopGainsShooter[1], pidTopGainsShooter[2]);
            shooter.low.setFeedforwardCoefficients(kBotGainsShooter[0], kBotGainsShooter[1], kBotGainsShooter[2]);
            shooter.high.setFeedforwardCoefficients(kTopGainsShooter[0], kTopGainsShooter[1], kTopGainsShooter[2]);

        }
    }


    //    @Override
//    public Command goToIntakeLine(){
//        return new SequentialCommandGroup(
//
//        );
//    }
    @Override
    protected Command postMotifSequence(){
        limelight.stop();//temporarily turn it off to hand to localizer
        return new SequentialCommandGroup(
                new WaitCommand(500),
                getToShootCommand(500),
                new WaitCommand(500),
                shootOptimal(motifPattern),
                //new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),

//                line1Commands(),
//                //new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 1000),
////                openGate(1000),
////
//                line2Commands(),
//                line3Commands(),
//
                park(100)
//
//               new InstantCommand(() -> spindexer.goTo(Angle.fromDegrees(0), CRServoEx2.RunMode.OptimizedPositionalControl))
//
        );
//        else{
//        return null;

    }


    protected Command line1Commands(){
        return new SequentialCommandGroup(
                getToLineNum(1, 1000),
                new WaitCommand(500),
//                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                new WaitCommand(500),
                intake(1, 0),
                setDefaultStartColors(),
                new WaitCommand(500),
                new InstantCommand(()-> intake.setDirectPower(0)),
                getToShootCommand(500),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                shootOptimal(motifPattern)
        );
    }
    protected Command line2Commands(){
        return new SequentialCommandGroup(
                getToLineNum(2, 1000),
                new WaitCommand(500),
//                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT1, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                new WaitCommand(500),
                intake(2, 1),
                setDefaultStartColors(),
                new WaitCommand(500),
                new InstantCommand(()-> intake.setDirectPower(0)),
                getToShootCommand(500),
                shootOptimal(motifPattern)
        );
    }
    protected Command line3Commands(){
        return new SequentialCommandGroup(
                getToLineNum(3, 1000),
                new WaitCommand(500),
//                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT2, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                new WaitCommand(500),
                intake(3, 2),
                setDefaultStartColors(),
                new WaitCommand(500),
                new InstantCommand(()-> intake.setDirectPower(0)),
                getToShootCommand(500),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                shootOptimal(motifPattern)
        );
    }
    private Command intakePower(long milliSec) {
        return new IntakeTimeCommand(intake, milliSec);
    }

    protected SequentialCommandGroup goToMotifDetection(long milliSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, motifDetectionPose, headingError, timeOutConstraint, pathDistThresholdMin),
                new WaitCommand(milliSec)
        );
    }
    protected SequentialCommandGroup setDefaultStartColors(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.setBallColors(startBallColors))
        );
    }
    protected SequentialCommandGroup openGate(long milliSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, openGatePose, headingError, timeOutConstraint, pathDistThresholdMin),
                new WaitCommand(milliSec)
        );
    }

    protected Command intake(int targetSpot, int initialSpindexerIntakeSpot){
//        autoIntakeCommand = new AutoIntakeCommand(spindexer, intake, intakePower, intakeTime);
        autoStart = true;
        return new ParallelCommandGroup(
                new InstantCommand(() -> intake.setDirectPower(0.8)),
                new SequentialCommandGroup(
                        new WaitCommand(1000),
                        new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex((initialSpindexerIntakeSpot + 1) % 3), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                        new InstantCommand(() -> spindexer.getTurner().setPIDFTOUse(spindexer.intakeTurnerCoeff)),
                        new WaitCommand(1000),
                        new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex((initialSpindexerIntakeSpot + 2) % 3), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0),
                        new InstantCommand(() -> spindexer.getTurner().setPIDFTOUse(spindexer.outtakeTurnerCoeff)),
                        new WaitCommand(1000),
                        new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex((initialSpindexerIntakeSpot + 2) % 3), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0)
                ),
                driveToIntakeEnd(targetSpot)
        );

//        return intakePower(milliSec);
    }
    protected SequentialCommandGroup driveToIntakeEnd(int spot){
        Pose intakePose = (spot == 1) ? intakeOnePose : (spot == 2) ? intakeTwoPose : intakeThreePose;

//        follower.update();
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, new Pose(intakePose.getX() + xChangeIntake, intakePose.getY(), intakePose.getHeading()), headingError, timeOutConstraint, pathDistThresholdMin)
                        .setMaxPower(0.3)
        );
    }

    protected SequentialCommandGroup park(long milliSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, parkPose, headingError, timeOutConstraint, pathDistThresholdMin),
                new WaitCommand(milliSec)
        );
    }
    protected SequentialCommandGroup startShoot(){
        return new SequentialCommandGroup(
                new ShootSeqCommand(spindexer, shooter, SpindexerSpot.convertFromindex(shootArray), follower, shootSide, false, TwoWheelShooter.ShootDist.Close, true)
        );
    }
    protected SequentialCommandGroup shootOptimal(MotifEnums.Motif pattern){
        BallColor[] colors = spindexer.getBallColors();
        BallColor[] PPG = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
        BallColor[] PGP = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};

        if (pattern == MotifEnums.Motif.GPP) {
            if (Arrays.equals(colors, PPG)) {
                spots = SpindexerSpot.convertFromindex(new int[]{2, 1, 0});
            } else if (Arrays.equals(colors, PGP)) {
                spots = SpindexerSpot.convertFromindex(new int[]{1, 2, 0});
            }
        }
        else if (pattern == MotifEnums.Motif.PGP) {
            if (Arrays.equals(colors, PPG)) {
                spots = SpindexerSpot.convertFromindex(new int[]{1, 2, 0});
            } else if (Arrays.equals(colors, PGP)) {
                spots = SpindexerSpot.convertFromindex(new int[]{2, 1, 0});
            }
        }
        else if (pattern == MotifEnums.Motif.PPG) {
            if (Arrays.equals(colors, PPG)) {
                spots = SpindexerSpot.convertFromindex(new int[]{1, 0, 2});
            } else if (Arrays.equals(colors, PGP)) {
                spots = SpindexerSpot.convertFromindex(new int[]{2, 0, 1});
            }
        }
        else {
            spots = SpindexerSpot.convertFromindex(new int[]{1, 0, 2});
        }

        seqShootCommand = new ShootSeqCommand(spindexer, shooter, spots, follower, shootSide, false, TwoWheelShooter.ShootDist.Close, true);
        return new SequentialCommandGroup(
                seqShootCommand
//                new InstantCommand(() -> shooter.setFlywheelsPower(TwoWheelShooter.ShootDist.Close)),
//                new SpindexerGotoSpot(spindexer, spots[0], SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 200),
//                new WaitCommand(2000),
//                new SpindexerGotoSpot(spindexer, spots[1], SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 200),
//                new WaitCommand(2000),
//                new SpindexerGotoSpot(spindexer, spots[2], SpotType.OUTTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 200),
//                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}))
        );
    }
    protected SequentialCommandGroup getToLineNum(int lineNum, long milliSec){
        SchedulePathTo command = null;
        if(lineNum == 1) command = new SchedulePathTo(follower, intakeOnePose, headingError, timeOutConstraint, pathDistThresholdMin);
        else if(lineNum == 2) command =  new SchedulePathTo(follower, intakeTwoPose, headingError, timeOutConstraint, pathDistThresholdMin);
        else command = new SchedulePathTo(follower, intakeThreePose, headingError, timeOutConstraint, pathDistThresholdMin);

        return new SequentialCommandGroup(
                command
        );
    }

    protected SequentialCommandGroup getToShootCommand(long millSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, shootPose, headingError, timeOutConstraint, pathDistThresholdMin).setMaxPower(0.7),
                new WaitCommand(millSec)
        );
    }


    protected void updateTelemetry(){
        // Update pose & follower
        follower.update();
        currentPose = follower.getPose();
        double currentTime = gameTimer.getTime();

        // Follower
        telemetry.addData("Current Voltage", shooter.getCurrVoltage());
        telemetry.addData("Ratio Voltage ", shooter.getTargetVoltage() / shooter.getCurrVoltage());

        telemetry.addData("Current Follower Pose", currentPose.getPose());
        telemetry.addData("Follower Velocity", follower.getVelocity());
        telemetry.addData("Start Ball Color 0", startBallColors[0]);
        telemetry.addData("Start Ball Color 1", startBallColors[1]);
        telemetry.addData("Start Ball Color 2", startBallColors[2]);

        telemetry.addData("New Ball Detected", spindexer.newBallDetected());
        if(seqShootCommand != null) {
            telemetry.addData("Seq Test Farthest Moved", seqShootCommand.farthestMoved);
        }

        telemetry.addData("Motif", motifPattern);
        if(spindexer.getBallColors() != null) {
            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
        }
        if(spots != null) {
            telemetry.addData("Spindexer Optimal Sequence 0", spots[0]);
            telemetry.addData("Spindexer Optimal Sequence 1", spots[1]);
            telemetry.addData("Spindexer Optimal Sequence 2", spots[2]);
        }
        telemetry.addData("All Occupied", spindexer.allOccuppiedBallColors());

        addToTelemGraph("Pose X", currentPose.getX());
        addToTelemGraph("Pose Y", currentPose.getY());
        addToTelemGraph("Pose Heading", currentPose.getHeading());
        addToTelemGraph("Follower T Value", follower.getCurrentTValue());
        addToTelemGraph("Follower Velocity X", follower.getVelocity().getXComponent());
        addToTelemGraph("Follower Velocity Y", follower.getVelocity().getYComponent());
        addToTelemGraph("Follower Velocity Mag", follower.getVelocity().getMagnitude());
        addToAllTelemGraph("Follower Velocity Heading", follower.getVelocity().getTheta());
        addToAllTelemGraph("Follower Translational Error", follower.getDriveError());
        addToAllTelemGraph("Follower Heading Error", follower.getHeadingError());
        addToTelemGraph("Follower Max Vel Constraint", follower.getConstraints().getVelocityConstraint());
        addToTelemGraph("Follower T Constraint", follower.getConstraints().getTValueConstraint());

        //Shooter
        if(shooter != null){
            addToAllTelemGraph("Shooter Low Flywheel Power", shooter.low.get());
            addToAllTelemGraph("Shooter High Flywheel Power", shooter.high.get());
            addToAllTelemGraph("Shooter Low Flywheel Vel", shooter.getPredictedBotVel());
            addToAllTelemGraph("Shooter High Flywheel Vel", shooter.getPredictedTopVel());
            telemetry.addData("Shooter Dir RunMode", shooter.runMode);
            telemetry.addData("Shooter RunMode", shooterRunMode);
            telemetry.addData("Shooter Low RunMode", shooter.low.motorEx.getMode());
            telemetry.addData("Shooter High RunMode", shooter.low.motorEx.getMode());
        }

        //Spindexer
        if(spindexer != null){
            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
            telemetry.addData("Balls Left", spindexer.getBallCount());
            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
        }

        //Intake
        if(intake != null){
            addToAllTelemGraph("Intake Power", intake.getMotor().get());
            addToAllTelemGraph("Intake Velocity", intake.getMotorVelocity());
        }

        //Time
        addToAllTelemGraph("Auto Elapsed Time", currentTime);
        addToAllTelemGraph("Update Rate", 1 / gameTimer.getDeltaTime());

        //Motif
        addBooleanToTelem("Motif Busy", !isVisionComplete());
        if(motifCommand != null){
            addStringToTelem("Motif Timer", String.valueOf(motifCommand.getTime()));
        }
        addStringToTelem("Motif Pattern", String.valueOf(motifPattern));

        //First Path
        if(firstPath != null){
            addBooleanToTelem("First Path Busy", !firstPath.isFinished());
        }
        if(seqShootCommand != null) {
            telemetry.addData("Spindexer Shoot CurrBallIndex", seqShootCommand.currBallIndex);
        }
        telemetry.addData("Spindexer Get Curr Angle", spindexer.getCurrentAngle());
//

        telemetry.update();
        graphManager.update();
        telemetryManager.update();
        if(dashboard!= null) {
            dashboard.sendTelemetryPacket(dashboardPacket);
        }
    }



    public void addStringToTelem(String s, String o){
        telemetry.addLine(s + o);
    }
    public void addToTelemGraph(String s, Number o){
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
    }
    public void addToAllTelemGraph(String s, Number o){
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
        telemetry.addData(s, o);
        if(dashboard != null) {
            dashboardPacket.put(s, o);
        };
    }
    public void addBooleanToTelem(String s, boolean o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
    }




}

