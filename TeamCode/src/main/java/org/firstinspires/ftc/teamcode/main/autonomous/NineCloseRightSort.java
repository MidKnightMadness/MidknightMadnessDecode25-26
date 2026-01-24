package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.shooter.ShootUpdateCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.readwrite.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Config
@Configurable
@Autonomous(name = "9 Close Right Sort", group = "Competition")
public class NineCloseRightSort extends BaseAuto {
    public static double motifDetectionTimeMs = 3000;
    int startPipeline = 1;
    public static Pose startPose = new Pose(118, 130, Math.toRadians(127));
    public static Pose motifDetectionPose = new Pose(87, 94, Math.toRadians(120));
    public static Pose shootPose = new Pose(87, 95, Math.toRadians(225));
    public static Pose parkPose = new Pose(85, 109, Math.toRadians(0));

    public static Pose openGatePose = new Pose(128, 69, Math.toRadians(0));

    public static Pose intakeOnePose = new Pose(100, 84, Math.toRadians(0));
    public static Pose intakeOneIntermediate = new Pose(87, 95, Math.toRadians(0));
    public static Pose intakeTwoPose = new Pose(100, 60, Math.toRadians(0));
    public static Pose intakeThreePose = new Pose(100, 36, Math.toRadians(0));
    public static Pose intakeOneEnd = new Pose(125, 84, Math.toRadians(0));
    public static Pose intakeTwoEnd= new Pose(125, 60, Math.toRadians(0));
    public static Pose intakeThreeEnd = new Pose(125, 36, Math.toRadians(0));
    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    MotifWriteCommand motifCommand = null;

    ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose;

    Command firstPath;
    public static long firstWaitTime = 700;
    public static long secondWaitTime = 500;
    public static long thirdWaitTime = 500;//250 old
    public static long fourthWaitTime = 500;

    public static int[] shootArray = new int[]{2, 1, 0};

    public static TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;

    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    SpindexerSpot[] spots;

    ShootSeqCommand seqShootCommand;
    public static TwoWheelShooter.ShootDist shootDist = TwoWheelShooter.ShootDist.Close;
    public static boolean voltageCompensation = true;
    public static boolean useLUT = false;
    Path toMotifDetection;
    Path toShootPresets;
    Path toIntakeThree;
    Path toIntakeEndThree;
    Path toShootOne;
    Path toIntakeTwo;
    Path toIntakeEndTwo;
    Path toShootTwo;
    Path toIntakeOne;
    Path toIntakeEndOne;
    Path toShootThree;
    Path toPark;
    Path toGate;
    public static boolean rawPowerOn = false;
    public static long shootOnTime = 4500;
    public static double pathDistThresholdMin = 0;
    public static double headingError = Math.toRadians(2);
    public static double timeOutConstraint = 100;
    public static double velConstraint = 0;

    @Override
    protected Pose getStartPose(){
        return startPose;
    }


    @Override
    protected void setupVision(){
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
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

        toMotifDetection = new Path(new BezierLine(startPose, motifDetectionPose));
        toMotifDetection.setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading());
        setConstraints(toMotifDetection);

//        toShootPresets = new Path(new BezierLine(motifDetectionPose, shootPose));
//        toShootPresets.setLinearHeadingInterpolation(motifDetectionPose.getHeading(), shootPose.getHeading());
//        setConstraints(toShootPresets);


//        setConstraints(toIntakeOne);

        toIntakeOne = new Path(new BezierLine(shootPose, intakeOnePose));
        toIntakeOne.setLinearHeadingInterpolation(shootPose.getHeading(), intakeOnePose.getHeading(), 0.8);
//        toIntakeEndOne = new Path(new BezierLine(intakeOnePose, intakeOneEnd));
//        toIntakeEndOne.setLinearHeadingInterpolation(intakeOnePose.getHeading(), intakeOneEnd.getHeading());
        setConstraints(toIntakeEndOne);

//        toGate = new Path(new BezierLine(intakeOneEnd, openGatePose));
//        toGate.setLinearHeadingInterpolation(intakeOneEnd.getHeading(), openGatePose.getHeading());
//        setConstraints(toGate);

//        toShootOne = new Path(new BezierLine(openGatePose, shootPose));
//        toShootOne.setLinearHeadingInterpolation(openGatePose.getHeading(), shootPose.getHeading());
//        setConstraints(toShootOne);

        toIntakeTwo = new Path(new BezierLine(shootPose, intakeTwoPose));
        toIntakeTwo.setLinearHeadingInterpolation(shootPose.getHeading(), intakeTwoPose.getHeading());
        setConstraints(toIntakeTwo);

//        toIntakeEndTwo = new Path(new BezierLine(intakeTwoPose, intakeTwoEnd));
//        toIntakeEndTwo.setLinearHeadingInterpolation(intakeTwoPose.getHeading(), intakeTwoEnd.getHeading());
//        setConstraints(toIntakeEndTwo);
//
//        toShootTwo = new Path(new BezierLine(intakeTwoEndPose, shootPose));
//        toShootTwo.setLinearHeadingInterpolation(intakeTwoEndPose.getHeading(), shootPose.getHeading());
//        setConstraints(toShootTwo);

        toIntakeThree = new Path(new BezierLine(shootPose, intakeThreePose));
        toIntakeThree.setLinearHeadingInterpolation(shootPose.getHeading(), intakeThreePose.getHeading());
        setConstraints(toIntakeThree);

//        toIntakeEndThree = new Path(new BezierLine(intakeThreePose, intakeThreeEnd));
//        toIntakeEndThree.setLinearHeadingInterpolation(intakeThreePose.getHeading(), intakeThreeEnd.getHeading());
//        setConstraints(toIntakeEndThree);

//        toShootThree = new Path(new BezierLine(intakeThreeEndPose, shootPose));
//        toShootThree.setLinearHeadingInterpolation(intakeThreeEndPose.getHeading(), shootPose.getHeading());
//        setConstraints(toShootThree);

        toPark = new Path(new BezierLine(shootPose, parkPose));
        toPark.setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading());
        setConstraints(toPark);

    }
    private void setConstraints(Path path){
        if(timeOutConstraint != 0) {
            path.setTimeoutConstraint(timeOutConstraint);
        }
        if(pathDistThresholdMin != 0) {
            path.setTranslationalConstraint(pathDistThresholdMin);
        }
        if(headingError != 0) {
            path.setHeadingConstraint(headingError);
        }
        if(velConstraint != 0){
            path.setVelocityConstraint(velConstraint);
        }
    }


    @Override
    protected boolean isVisionComplete(){
        if(motifCommand.getDetected() != MotifEnums.Motif.NONE){
            motifPattern = motifCommand.getDetected();
        }
        if(motifCommand.isFinished()){
            return true;
        }
        return false;
    }

    AutoIntakeCommand autoIntakeCommand;
    boolean autoStart = false;
    int currSpindexerGotoSpot = -1;
    public static double spindexerSpeed = -0.20;
    public static double headingThreshold = Math.toRadians(2);

    @Override
    protected Command preMotifSequence(){
        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
        firstPath = new FollowPathCommand(follower, toMotifDetection, false).setGlobalMaxPower(1.0);
        return new SequentialCommandGroup(
                firstPath,
                motifCommand
        );
    }

    @Override
    public void update(){
        if(currSpindexerGotoSpot != -1){
            spindexer.goToSpot(SpindexerSpot.fromIndex(currSpindexerGotoSpot), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        }
    }
    @Override
    protected void initializeMechanisms() {
        spindexer = new Spindexer(hardwareMap, false).setBallColors(startBallColors).initAngle();
        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
        register(intake, shooter, spindexer);
    }

    @Override
    protected Command postMotifSequence(){
        limelight.stop();
        limelight.shutdown();
        //temporarily turn it off to hand to localizer
        return new SequentialCommandGroup(
                setSpindexerCorrect(0),
                shoot(0),

                getToLineNum(1),
                intakeLineOne(),
                shoot(3),
//                openGate()

                getToLineNum(2),
                intakeLineTwo(),
                shoot(2),
//
//                getToLineNum(3),
//                intakeLineThree(),
//                shoot(3)
                new ParallelCommandGroup(
                    new InstantCommand(()-> currSpindexerGotoSpot = 0),
                    park()
                )

        );

    }
    protected Command setSpindexerCorrect(int lineNum){
        if(lineNum == 0) {
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 2),
                            () -> motifPattern == MotifEnums.Motif.PGP
                    ),
                    () -> motifPattern == MotifEnums.Motif.GPP
            );
        } else if(lineNum == 3){
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            () -> motifPattern == MotifEnums.Motif.PPG
                    ),
                    () -> motifPattern == MotifEnums.Motif.GPP
            );
        }  else if(lineNum == 2) {
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            () -> motifPattern == MotifEnums.Motif.PGP
                    ),
                    () -> motifPattern == MotifEnums.Motif.PPG
            );
        }
        else{//lineNum = 1
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            () -> motifPattern == MotifEnums.Motif.PPG
                    ),
                    () -> motifPattern == MotifEnums.Motif.PGP
            );
        }
    }

    protected Command intakeLineOne(){
        return new SequentialCommandGroup(
//                new InstantCommand(()-> currSpindexerGotoSpot = 0),
                intake(1, 0)
        );
    }

    protected Command intakeLineTwo(){
        return new SequentialCommandGroup(
//                new InstantCommand(()-> currSpindexerGotoSpot = 0),
                intake(2, 0)
        );
    }

    protected Command intakeLineThree(){
        return new SequentialCommandGroup(
//                new InstantCommand(()-> currSpindexerGotoSpot = 0),
                intake(3, 0)
        );
    }
    protected Command shoot(int shootNum){
        return new SequentialCommandGroup(
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        new ShootUpdateCommand(spindexer, shooter, follower, shootSide, useLUT, voltageCompensation, shootDist, rawPowerOn).withTimeout(shootOnTime),
                        new SequentialCommandGroup(
                                getToShootCommand(shootNum),
                                new InstantCommand(() -> currSpindexerGotoSpot = -1),
//                                new InstantCommand(() -> spindexer.getTurner().setRunMode(CRServoEx2.RunMode.RawPower)),
//                                new InstantCommand(() -> spindexer.getTurner2().setRunMode(CRServoEx2.RunMode.RawPower)),
                                new WaitCommand(3000),
                                new InstantCommand(() -> spindexer.spin(1 * spindexerSpeed))
                        )
                ),
                new ParallelCommandGroup(
                        new InstantCommand(()-> shooter.stopFlywheels()),
                        //new InstantCommand(() -> spindexer.getTurner2().getServo().setPower(0)),
                        new InstantCommand(() -> spindexer.getTurner().getServo().setPower(0))
                ),
//                new InstantCommand(() -> spindexer.getTurner().setRunMode(CRServoEx2.RunMode.OptimizedPositionalControl)),
//                new InstantCommand(() -> spindexer.getTurner2().setRunMode(CRServoEx2.RunMode.OptimizedPositionalControl)),
                new InstantCommand(() -> currSpindexerGotoSpot = 0)
//                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}))
//        );
        );
    }

    protected SequentialCommandGroup setDefaultStartColors(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.setBallColors(startBallColors))
        );
    }
//    protected SequentialCommandGroup openGate(long milliSec){
//        return new SequentialCommandGroup(
//                new SchedulePathTo(follower, openGatePose, headingError, timeOutConstraint, pathDistThresholdMin),
//                new WaitCommand(milliSec)
//        );
//    }


    protected Command intake(int targetSpot, int initialSpindexerIntakeSpot){
        return new SequentialCommandGroup(
                new InstantCommand(()-> intake.setDirectPower(1.0)),
                new ParallelCommandGroup(
                        //new AutoIntakeCommand(spindexer, intake, 1.0, 20000, inBetweenTime),
                        new SequentialCommandGroup(
                                new WaitCommand(firstWaitTime),
                                new InstantCommand(() -> currSpindexerGotoSpot = 1),
                                new WaitCommand(secondWaitTime),
                                new InstantCommand(() -> currSpindexerGotoSpot = 2),
                                new WaitCommand(thirdWaitTime)
                        ),
                        driveToIntakeEnd(targetSpot)
                ).withTimeout(4500),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setDirectPower(0)),
                        setSpindexerCorrect(targetSpot)
                )
        );
    }

    protected SchedulePathTo driveToIntakeEnd(int spot){
        if(spot == 3) {
            return new SchedulePathTo(follower, intakeThreePose, intakeThreeEnd, headingThreshold).setMaxPower(0.3);
        } else if(spot == 2){
            return new SchedulePathTo(follower, intakeTwoPose, intakeTwoEnd, headingThreshold).setMaxPower(0.3);
        } else {
            return new SchedulePathTo(follower, intakeOnePose, intakeOneEnd, headingThreshold).setMaxPower(0.3);
        }


    }

    protected FollowPathCommand park(){
        return new FollowPathCommand(follower, toPark, true, 1.0);
    }

    protected SequentialCommandGroup getToLineNum(int lineNum){
        SequentialCommandGroup command = null;
        if(lineNum == 3){
            return new SequentialCommandGroup(
                new FollowPathCommand(follower, toIntakeThree, true)
            );
        }
        if(lineNum == 2){
            return new SequentialCommandGroup(
                    new FollowPathCommand(follower, toIntakeTwo, true)
            );
        }
        else {
            return new SequentialCommandGroup(
                    new FollowPathCommand(follower, toIntakeOne, true)
            );
        }
    }

    protected SchedulePathTo getToShootCommand(int num){
        Pose currPose;
        if(num == 0){
            currPose = startPose;
        } else if(num == 1){
            currPose = intakeThreeEnd;
        } else if(num == 2){
            currPose = intakeTwoEnd;
        } else{
            currPose = intakeOneEnd;
        }
        return new SchedulePathTo(follower, currPose, shootPose, headingThreshold).setMaxPower(1.0);
    }


    protected void updateTelemetry(){
        // Update pose & follower
        follower.update();
        currentPose = follower.getPose();
//        double currentTime = gameTimer.getTime();

        // Follower
        telemetry.addData("Update Rate", 1000.0 / gameTimer.getDeltaTime());
        telemetry.addData("Curr Spindexer GotoSpot", currSpindexerGotoSpot);
        telemetry.addData("Current Voltage", shooter.getCurrVoltage());
        telemetry.addData("Ratio Voltage ", shooter.getTargetVoltage() / shooter.getCurrVoltage());

        telemetry.addData("Current Follower Pose", currentPose.getPose());
        telemetry.addData("Follower Velocity", follower.getVelocity());
        telemetry.addData("Start Ball Color 0", startBallColors[0]);
        telemetry.addData("Start Ball Color 1", startBallColors[1]);
        telemetry.addData("Start Ball Color 2", startBallColors[2]);

//        telemetry.addData("New Ball Detected", spindexer.newBallDetected());

        telemetry.addData("Motif", motifPattern);
        if(spindexer.getBallColors() != null) {
            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
        }
//        if(spots != null) {
//            telemetry.addData("Spindexer Optimal Sequence 0", spots[0]);
//            telemetry.addData("Spindexer Optimal Sequence 1", spots[1]);
//            telemetry.addData("Spindexer Optimal Sequence 2", spots[2]);
//        }
//        telemetry.addData("All Occupied", spindexer.allOccuppiedBallColors());
//
        //Shooter
        if(shooter != null){

            telemetry.addData("Shooter Low Vel", shooter.low.getVelocity());
            telemetry.addData("Shooter High Vel", shooter.high.getVelocity());
            telemetry.addData("Shooter Dir RunMode", shooter.runMode);
            telemetry.addData("Shooter RunMode", shooterRunMode);
            telemetry.addData("Shooter Low RunMode", shooter.low.motorEx.getMode());
            telemetry.addData("Shooter High RunMode", shooter.low.motorEx.getMode());
        }

        //Spindexer
        if(spindexer != null){
            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
//            telemetry.addData("Balls Left", spindexer.getBallCount());
            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
        }

        //Motif
        telemetryManager.addData("Motif Pattern", motifPattern);

        telemetry.addData("Spindexer Get Curr Angle", spindexer.getCurrentAngle());
//

        telemetry.update();
//        graphManager.update();
//        telemetryManager.update();
//        if(dashboard!= null) {
//            dashboard.sendTelemetryPacket(dashboardPacket);
//        }
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