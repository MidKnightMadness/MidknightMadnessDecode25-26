package org.firstinspires.ftc.teamcode.main.autonomous;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.ShootUpdateCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeTimeCommand;
import org.firstinspires.ftc.teamcode.commands.readwrite.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoSpot;
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
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;

@Config
@Configurable
@Autonomous(name = "9 Far Left Sort", group = "Competition")
public class NineBackLeftSort extends BaseAuto {
    int startPipeline = 1;
    public static Pose startPose = new Pose(56, 8, Math.toRadians(90));
    public static Pose shootPose = new Pose(60, 17, Math.toRadians(295));
    public static Pose forwardPose = new Pose(56, 12, Math.toRadians(90));
    public static Pose parkPose = new Pose(58, 38, Math.toRadians(180));
    public static Pose openGatePose = new Pose(8, 76, Math.toRadians(180));
    public static Pose intakeOnePose = new Pose(43.5, 84, Math.toRadians(180));
    public static Pose intakeTwoPose = new Pose(43.5, 58, Math.toRadians(180));
    public static Pose intakeThreePose = new Pose(43.5, 36, Math.toRadians(180));
    public static Pose intakeOneEnd = new Pose(19, 84, Math.toRadians(180));
    public static Pose intakeTwoEnd= new Pose(19, 58, Math.toRadians(180));
    public static Pose intakeThreeEnd = new Pose(12, 36, Math.toRadians(180));
    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    MotifWriteCommand motifCommand = null;

    ShootSide shootSide = ShootSide.LEFT;
    Pose currentPose;

    Command firstPath;
    public static long firstWaitTime = 700;
    public static long secondWaitTime = 500;
    public static long thirdWaitTime = 500;//250 old

    public static long fourthWaitTime = 500;
    public static double pathDistThresholdMin = 0;
    public static double headingError = 0;
    public static double timeOutConstraint = 0;
    public static double velConstraint = 0;

//    public static double pathDistThresholdMin = 1.5;
//    public static double headingError = Math.toRadians(2);
//    public static double timeOutConstraint = 1000;

    //TODO: TRY VELOCITY CONSTRAINT
    public static TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.RawPower;

    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    SpindexerSpot[] spots;

    public static TwoWheelShooter.ShootDist shootDist = TwoWheelShooter.ShootDist.Far;
    public static boolean voltageCompensation = true;//TODO:TRY FALSE
    public static boolean useLUT = false;
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
    @Override
    protected Pose getStartPose(){
        return startPose;
    }

    @Override
    protected void setupVision(){
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        limelight.pipelineSwitch(startPipeline);
        limelight.start();
        file = createFile(fileName, directoryName);

        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            RobotLog.ee("Log", "Error instantiating file writer of file: " + e.getMessage());
        }
    }

    @Override
    protected ShootSide getSide(){
        return shootSide;
    }

    String fileName = "motif_value.txt";
    String directoryName = "competition";
    FileWriter fileWriter;
    File file;
    boolean finishedWritingMotif = false;
    public static boolean useDistanceSensor = false;
    public static double inBetweenTime = 100;
    public static boolean rawPowerOn = false;
    public static long shootOnTime = 5000;
    int aprilTagID = 0;
    @Override
    public void initialize_loop(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
            if(list.size() != 0) {
                LLResultTypes.FiducialResult item = list.get(0);
                aprilTagID = item.getFiducialId();
                motifPattern = idMap.getOrDefault(aprilTagID, MotifEnums.Motif.NONE);
            }
        }
    }

    @Override
    public void writeMotif(){
        if (motifPattern != MotifEnums.Motif.NONE) {
            writeToFile(fileWriter, String.valueOf(aprilTagID));
            closeFileWriter(fileWriter);
            finishedWritingMotif = true;
        }
    }


    private static File createFile(String fileName, String dirName){
        File dir = new File(Environment.getExternalStorageDirectory(), dirName);
        if(!dir.exists()){
            dir.mkdirs();
        }
        File file = new File(dir, fileName);
        return file;
    }


    private void writeToFile(FileWriter fileWriter, String s){
        try {
            fileWriter.write(s);
            fileWriter.flush();
        } catch (IOException e) {
            RobotLog.ee("Log", "No file writer detected: " + e.getMessage());
        }
    }

    private void closeFileWriter(FileWriter fileWriter){
        try {
            fileWriter.close();
        } catch (IOException e) {
            RobotLog.ee("Log", "Cannot close file writer: " + e.getMessage());
        }
    }

    Map<Integer, MotifEnums.Motif> idMap = Map.of(
            21, MotifEnums.Motif.GPP,
            22, MotifEnums.Motif.PGP,
            23, MotifEnums.Motif.PPG
    );
    //keep these empty and build the path using follower's current Pose


    @Override
    protected void buildPaths(){
        toShootPresets = new Path(new BezierLine(forwardPose, shootPose));
        toShootPresets.setLinearHeadingInterpolation(forwardPose.getHeading(), shootPose.getHeading());
        setConstraints(toShootPresets);

        toIntakeThree = new Path(new BezierLine(shootPose, intakeThreePose));
        toIntakeThree.setLinearHeadingInterpolation(shootPose.getHeading(), intakeThreePose.getHeading());
        setConstraints(toIntakeThree);

//        toIntakeEndThree = new Path(new BezierLine(intakeThreePose, intakeThreeEnd));
//        toIntakeEndThree.setLinearHeadingInterpolation(intakeThreePose.getHeading(), intakeThreeEnd.getHeading());
////        setConstraints(toIntakeEndThree);

//        toShootOne = new Path(new BezierLine(intakeThreeEnd, shootPose));
//        toShootOne.setLinearHeadingInterpolation(intakeThreeEnd.getHeading(), shootPose.getHeading());
//        setConstraints(toShootOne);

        toIntakeTwo = new Path(new BezierLine(shootPose, intakeTwoPose));
        toIntakeTwo.setLinearHeadingInterpolation(shootPose.getHeading(), intakeTwoPose.getHeading());
        setConstraints(toIntakeTwo);

//        toIntakeEndTwo = new Path(new BezierLine(intakeTwoPose, intakeTwoEnd));
//        toIntakeEndTwo.setLinearHeadingInterpolation(intakeTwoPose.getHeading(), intakeTwoEnd.getHeading());
////        setConstraints(toIntakeEndTwo);

//        toShootTwo = new Path(new BezierLine(intakeTwoEnd, shootPose));
//        toShootTwo.setLinearHeadingInterpolation(intakeTwoEnd.getHeading(), shootPose.getHeading());
////        setConstraints(toShootTwo);

        toIntakeOne = new Path(new BezierLine(shootPose, intakeOnePose));
        toIntakeOne.setLinearHeadingInterpolation(shootPose.getHeading(), intakeOnePose.getHeading());
        setConstraints(toIntakeOne);

//   /     setConstraints(toIntakeEndOne);

//        toShootThree = new Path(new BezierLine(intakeOneEnd, shootPose));
//        toShootThree.setLinearHeadingInterpolation(intakeOneEnd.getHeading(), shootPose.getHeading());
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
        return true;
    }

    AutoIntakeCommand autoIntakeCommand;
    boolean autoStart = false;
    int currSpindexerGotoSpot = -1;
    public static double spindexerSpeed = -0.10;

    @Override
    protected Command preMotifSequence(){
        return new InstantCommand();
    }

    @Override
    public void update(){
        if(currSpindexerGotoSpot != -1) {
            spindexer.goToSpot(SpindexerSpot.fromIndex(currSpindexerGotoSpot), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        }
    }
    @Override
    protected void initializeMechanisms() {
        spindexer = new Spindexer(hardwareMap, useDistanceSensor).setBallColors(startBallColors).initAngle();
//        spindexer.getTurner2().setRunMode(CRServoEx2.RunMode.OptimizedPositionalControl);
//        spindexer.getTurner().setRunMode(CRServoEx2.RunMode.OptimizedPositionalControl);
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
//                driveForward(),
                setSpindexerCorrect(0),
                new WaitCommand(1000),
                shoot(0),

                getToLineNum(3),
                intakeLineThree(),
                shoot(1),

                getToLineNum(2),
                intakeLineTwo(),
                shoot(2),
//                getToLineNum(1),
//                intakeLineOne(),
//                shoot(3),
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
                //new InstantCommand(()-> currSpindexerGotoSpot = 0),
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
                                new WaitCommand(1000),
                                new InstantCommand(() -> spindexer.spin(1 * spindexerSpeed))
                        )
                ),
                new ParallelCommandGroup(
                        new InstantCommand(()-> shooter.stopFlywheels()),
                        new InstantCommand(() -> spindexer.getTurner2().getServo().setPower(0)),
                        new InstantCommand(() -> spindexer.getTurner().getServo().setPower(0))
                ),
//                new InstantCommand(() -> spindexer.getTurner().setRunMode(CRServoEx2.RunMode.OptimizedPositionalControl)),
//                new InstantCommand(() -> spindexer.getTurner2().setRunMode(CRServoEx2.RunMode.OptimizedPositionalControl)),
                new InstantCommand(() -> currSpindexerGotoSpot = 0)
                //TODO: HERE
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
                                //new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(0), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 500),
                                new WaitCommand(firstWaitTime),
                                new InstantCommand(() -> currSpindexerGotoSpot = 1),
                                //new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(1), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 500),
                                new WaitCommand(secondWaitTime),
                                new InstantCommand(() -> currSpindexerGotoSpot = 2),
                                //new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(2), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 500),
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
            return new SchedulePathTo(follower, intakeThreePose, intakeThreeEnd).setMaxPower(0.3);
        } else if(spot == 2){
            return new SchedulePathTo(follower, intakeTwoPose, intakeTwoEnd).setMaxPower(0.3);
        } else {
            return new SchedulePathTo(follower, intakeOnePose, intakeOneEnd).setMaxPower(0.3);
        }
    }

    protected FollowPathCommand park(){
        return new FollowPathCommand(follower, toPark, true, 1.0);
    }

    protected FollowPathCommand getToLineNum(int lineNum){
        FollowPathCommand command = null;
        if(lineNum == 3) command = new FollowPathCommand(follower, toIntakeThree, true);
        else if(lineNum == 2) command = new FollowPathCommand(follower, toIntakeTwo, true);
        else command = new FollowPathCommand(follower, toIntakeOne, true);

        return command;
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
        return new SchedulePathTo(follower, currPose, shootPose).setMaxPower(1.0);
    }

    @Override
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
//        telemetryManager.addData("Motif Pattern", motifPattern);

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
