package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.util.ExtraFns.normAngle;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.SpindexerGotoAngle;
import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.old.opModes.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.commands.ShootHardcode;
import java.io.File;
import java.util.Map;

@Config
@Configurable
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {
    Follower follower;
    Pose startPose = new Pose(72, 8, Math.toRadians(90));
    Pose currentPose;
    Timer timer;

    Spindexer spindexer;
    TwoWheelShooter shooter;
    Intake intake;


    MotifEnums.Motif pattern = MotifEnums.Motif.NONE;
    Map<String, MotifEnums.Motif> idMap = Map.of(
            "21", MotifEnums.Motif.GPP,
            "22", MotifEnums.Motif.PGP,
            "23", MotifEnums.Motif.PPG
    );
    String motifFileName = "competition/motif_value.txt";
    String botXFileName = "competition/robot_x.txt";
    String botYFileName = "competition/robot_y.txt";
    String botHeadingFileName = "competition/robot_heading.txt";
    String sideFileName = "competition/side.txt";

    double currSpeed = 0.8;
    double maxSpeed = 0.8;
    double midSpeed = 0.5;
    double intakePower = 0.6;
    public static double maxIntakePower = 1;
    double intakeTargetVel = 3;
    public static double headingAlignThresholRad = Math.toRadians(1.5);
    TelemetryManager telemetryM;
    GraphManager graphM;

    public static double autoIntakePower = 1;
    public static double[] pidBotGainsShooter = new double[]{0.0004, 0, 0.00001};
    public static double[] kBotGainsShooter = new double[]{0, 0.00005, 0};
    public static double[] pidTopGainsShooter = new double[]{0.0004, 0, 0.00001};
    public static double[] kTopGainsShooter = new double[]{0.02, 0.00005, 0};
    public static double[] pidIntakeGains = new double[]{0.0004, 0.0, 0.00001};
    public static double[] kIntakeGains = new double[]{0, 0, 0};
    double topShooterPower = 0.8;
    double botShooterPower = 0.6;
    Pose toCloseLeftShoot = new Pose(57, 94, Math.toRadians(310));
    Pose toCloseRightShoot =  new Pose(87, 94, Math.toRadians(230));
    Pose toFarLeftShoot = new Pose(67, 11, Math.toRadians(110));
    Pose toFarRightShoot = new Pose(5, 11, Math.toRadians(70));
    ShootSide shootSide = ShootSide.LEFT;
    CRServo spindexerServo;

    public static double currturnerSpeed = 0.3;
    double maxTurnerSpeed = 1;

    Pose leftTarget = new Pose(0, 144, Math.toRadians(45));
    Pose rightTarget = new Pose(144, 144, Math.toRadians(-45));
    ShootHardcode spindexerHardcode;

    boolean autoAlign = false;
    boolean autoSpindexer = false;
    boolean autoDriveToShoot = false;
    boolean autoIntake = false;
    boolean autoShootSeq = false;

    public static double spindexerCompensationOffset = Math.toRadians(5);//degrees
    int spindexerDirection;

    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;

    public static double[] pidAutoAlign = new double[]{0.7, 0, 0.2};//1.5, 0, 0.1
    public static double alignmentWeight = 0.2;
    FollowPathCommand followPathCommand;
    double prevHeadingError = 0;
    double turnPower;
    double headingError;
    public static Intake.RunMode intakeRunMode = Intake.RunMode.RawPower;
    public static CRServoEx2.RunMode spindexerRunMode = CRServoEx2.RunMode.OptimizedPositionalControl;
//    public static double spindexerFinishedTimeThreshold = 300;//mms
//    SpindexerGotoSpot spindexerSpotCommand;
    SpindexerGotoSpot spindexerGotoSpot;
    public static double change = 1;


    int currentIntakeSpot = 0;

//    public static double targetTopVel = 1000;
//    public static double targetBotVel = 1000;
    public static double shootAngleTolerance = 10;
    ShootSeqCommand seqCommand;
    SpindexerGotoSpot goToSpotCommand;
    AutoIntakeCommand autoIntakeCommand;
    boolean mapDistToShoot = true;

    Telemetry dashboardTelemetry;
    TwoWheelShooter.ShootDist currentShootDist;
    boolean useLUT = false;


    public static double pathDistThresholdMax = 0.5;
    public static double headingErrorMax = 0.025;
    public static double timeOutConstraint = 200;

    boolean spindexerGoingToSpot = false;
    int goToSpotCurrNum = 0;
    SpotType gotToSpotType = SpotType.INTAKE;

    BallColor[] defaultBallColor = new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    @Override
    public void initialize() {
//        CommandScheduler.getInstance().setBulkReading(
//                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
//        );
        CommandScheduler.getInstance().cancelAll();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();

//       ConstantsBot.motifIsBusy = false; -> meant for pinpointAprilTagLocalizer
        timer = new Timer();

        pattern = readMotifFromFile(motifFileName);
        double robotX = readDoubleFromPose(botXFileName);
        double robotY = readDoubleFromPose(botYFileName);
        double robotHeading = readDoubleFromPose(botHeadingFileName);
        shootSide = readShootSideFromFile(sideFileName);

        Pose roboPose = new Pose(robotX, robotY, robotHeading);
        startPose = roboPose != null ? roboPose : startPose;

        currentShootDist = (startPose.getY() > 20) ? TwoWheelShooter.ShootDist.Close : TwoWheelShooter.ShootDist.Far;

        follower = ConstantsBot.createPinpointFollowerCustom(hardwareMap, new Pose(0, 0, 0));
        follower.setPose(startPose);

        initializeSubsystems();


        follower.startTeleopDrive();
        spindexerServo = spindexer.getTurner().getServo();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        goToSpotCommand = new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0);
        seqCommand = new ShootSeqCommand(spindexer, shooter, new SpindexerSpot[]{SpindexerSpot.SPOT0, SpindexerSpot.SPOT1, SpindexerSpot.SPOT2}, follower, shootSide, mapDistToShoot, TwoWheelShooter.ShootDist.Close, true);
        spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});


        updateStartShootDist();
        register(intake, shooter, spindexer);
    }

    public void initializeSubsystems(){
        spindexer = new Spindexer(hardwareMap, false, new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        spindexer.initAngle(Angle.fromDegrees(0));
        spindexer.setMode(spindexerRunMode);
        spindexer.getTurner().stop();

        intake = new Intake(hardwareMap, intakeRunMode);
        if(intakeRunMode == Intake.RunMode.VelocityControl){
            resetVelocityGains(intake, pidIntakeGains, kIntakeGains);

        }

        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
//        shooter.setRunMode(shooterRunMode);
        if(shooterRunMode == TwoWheelShooter.RunMode.VelocityControl) {
            resetVelocityGains(shooter, pidBotGainsShooter, kBotGainsShooter, pidTopGainsShooter, kTopGainsShooter);
        }
        shooter.low.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.high.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.low.stopAndResetEncoder();
        shooter.high.stopAndResetEncoder();

    }


    public void resetVelocityGains(Intake intake, double[] pidIntakeGains, double[] kIntakeGains){
        intake.setPid(pidIntakeGains[0], pidIntakeGains[1], pidIntakeGains[2]);
        intake.setFeedforward(kIntakeGains[0], kIntakeGains[1], kIntakeGains[2]);
    }
    public void resetVelocityGains(TwoWheelShooter shooter, double[] pidBotGains, double[] kBotGains, double[] pidTopGains, double[] kTopGains){
        shooter.low.setVeloCoefficients(pidBotGains[0], pidBotGains[1], pidBotGains[2]);
        shooter.low.setFeedforwardCoefficients(kBotGains[0], kBotGains[1], kBotGains[2]);
        shooter.high.setVeloCoefficients(pidTopGains[0], pidTopGains[1], pidTopGains[2]);
        shooter.high.setFeedforwardCoefficients(kTopGains[0], kTopGains[1], kTopGains[2]);
    }

    public MotifEnums.Motif readMotifFromFile(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(),  fileName);
        if(file.exists()){
            return idMap.get(ReadWriteFile.readFile(file));
        }
        else{
            telemetry.addData("File does not exist", file.getAbsolutePath());
        }
        return MotifEnums.Motif.NONE;
    }

    public double readDoubleFromPose(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if(file.exists()) {
            return Double.parseDouble(ReadWriteFile.readFile(file));
        }
        else {
            return 0;
        }
    }
    public ShootSide readShootSideFromFile(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if(file.exists()) {
            return ReadWriteFile.readFile(file).equals("Left") ? ShootSide.LEFT : ShootSide.RIGHT;
        }
        else {
            return ShootSide.LEFT;
        }
    }
    public void addDataTelemetryGraph(String key, Number value) {
        telemetry.addData(key, value);
        telemetryM.addData(key, value);
        graphM.addData(key, value);
    }

    public void updateStartShootDist(){
        if(currentPose != null) {
            currentShootDist = (startPose.getY() > 20) ? TwoWheelShooter.ShootDist.Close : TwoWheelShooter.ShootDist.Far;
        }
    }


    @Override
    public void run(){
        super.run();
        runGamepad1Comands();
        runGamepad2Commands();
        emergencyStops();
        follower.update();

        updateTelem();


    }

    private void emergencyStops() {
        if(gamepad2.dpadUpWasPressed()) {
            if (autoSpindexer) {
                if (spindexerGotoSpot != null) {
//                    spindexerGotoSpot.end(true);
                    CommandScheduler.getInstance().cancel(spindexerGotoSpot);
                }
                //trying to turn off spindexer
//                spindexer.getTurner().setRunMode(CRServoEx2.RunMode.RawPower);
                spindexer.getTurner().setRunMode(CRServoEx2.RunMode.RawPower);
                spindexerServo.setPower(0);
                autoSpindexer = false;
            }

            if (autoIntake) {
                if (goToSpotCommand != null && !goToSpotCommand.isFinished()) {
                    CommandScheduler.getInstance().cancel(goToSpotCommand);
//                    goToSpotCommand.end(true);
                }
                if (autoIntakeCommand != null && !autoIntakeCommand.isFinished()) {
                    CommandScheduler.getInstance().cancel(autoIntakeCommand);
//                    autoIntakeCommand.end(true);
                }
//                spindexer.getTurner().setRunMode(CRServoEx2.RunMode.RawPower);
                autoIntake = false;
            }

            if (autoShootSeq) {
                if(seqCommand != null && !seqCommand.isFinished()){
                    CommandScheduler.getInstance().cancel(seqCommand);
//                    seqCommand.end(true);
                }
                spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
                autoShootSeq = false;
            }
        }
    }


    public PathChain getPathChain(Pose pose1, Pose pose2){
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(pose1.getPose(), pose2.getPose()))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .setHeadingConstraint(headingErrorMax)
                .setTimeoutConstraint(timeOutConstraint)
                .setTranslationalConstraint(pathDistThresholdMax)
                .build();
        return pathChain;
    }

    private void autoSwapShootSide(Follower follower){
        double heading = follower.getPose().getHeading();
        //convert to -180 - 180
        heading = (heading + 180 ) % 360 -180;
        if(heading < 0){
            shootSide = ShootSide.RIGHT;
        }
        else{
            shootSide = ShootSide.LEFT;
        }
    }

    private double normAnglePlusMinus2PI(double error){
        while(error < -Math.PI *2){
            error += Math.PI *2;
        }
        while(error > Math.PI * 2){
            error -= Math.PI * 2;
        }
        return error;
    }


    //calculates right rotation power to get heading to right pose
    private double calculateGamepadPID(double prevHeadingError, double headingError){
//        double filteredHeadingError = (1-alignmentWeight) * headingError + alignmentWeight * prevHeadingError;
        double filteredHeadingError = headingError;
        double pGain = pidAutoAlign[0] * filteredHeadingError;
        double dGain = pidAutoAlign[2] * (filteredHeadingError - prevHeadingError) / timer.getDeltaTime();

        double power = pGain + dGain;
        power = Math.max(-1, Math.min(1, power));
        if(Math.abs(filteredHeadingError) < headingAlignThresholRad){
            return 0;
        }

        return power;
    }
    public double getAngleError(Pose position, Pose target){
        double deltaY = target.getY() - position.getY();
        double deltaX = target.getX() - position.getX();
        double heading = Math.atan2(deltaY, deltaX);
        heading = normAngle(heading);
        //heading is in absolute degrees
        double error = heading - position.getHeading();
        double errorSign = (error > 0 ) ? -1 : 1;
        if(Math.abs(error) > Math.PI){
            error = errorSign * (2 * Math.PI - Math.abs(position.getHeading() - heading));
        }

        error = normAnglePlusMinus2PI(error);
        return error;
    }
    private double convertRadToDegrees(double val){
        return val * 180 / Math.PI;
    }

    private void setAlignTurnPower(){
        turnPower = -gamepad1.right_stick_x * currSpeed;
        //MODIFY so that the heading is facing the outake side, not the intake side
        Pose outakeSide = new Pose(follower.getPose().getX(), follower.getPose().getY(), normAngle(follower.getHeading() + Math.PI));
        //add compensation for spindexer direction
        double distToTarget = getDistance(follower.getPose(), outakeSide);
        double compY = outakeSide.getY() + spindexerDirection * distToTarget * Math.tan(spindexerCompensationOffset);

        Pose compensatedPose = new Pose(outakeSide.getX(), compY, outakeSide.getHeading());
        headingError = getAngleError(outakeSide, ((shootSide == ShootSide.LEFT) ? leftTarget : rightTarget));
        if(autoAlign) {
            turnPower = calculateGamepadPID(prevHeadingError, headingError);
        }
        prevHeadingError = headingError;
    }

    public static double getDistance(Pose start, Pose target){
        double dist = Math.sqrt(Math.pow(Math.abs(target.getY() - start.getY()), 2) + Math.pow(Math.abs(target.getX() - start.getX()), 2));
        return dist;
    }
    private void runGamepad1Comands(){
        currentPose = follower.getPose();

        setAlignTurnPower();
        manualChangeShootSide();
        driveRobot();//includes automations
        manualChangeMotif();
        //change min/max speed
        if (gamepad1.rightBumperWasPressed()) {
            currSpeed = currSpeed == maxSpeed ? midSpeed : maxSpeed;
        }

        //change autoAlignment to the goal
        if(gamepad1.leftBumperWasPressed()){
            autoAlign = !autoAlign;
        }

    }

    private void manualChangeShootSide(){
        if(gamepad1.xWasPressed()){
            shootSide = shootSide == ShootSide.LEFT ? ShootSide.RIGHT : ShootSide.LEFT;
        }
    }
    //manual changing of motif pattern - in case was not detected
    private void manualChangeMotif(){
        if(gamepad1.bWasPressed()){
            if(pattern == MotifEnums.Motif.NONE){
                pattern = MotifEnums.Motif.PGP;
            }
            else if(pattern == MotifEnums.Motif.PGP){
                pattern = MotifEnums.Motif.PPG;
            }
            else if(pattern == MotifEnums.Motif.PPG){
                pattern = MotifEnums.Motif.GPP;
            }
            else{
                pattern = MotifEnums.Motif.NONE;
            }
        }
    }

    public void setBallColorsDefault(){
        if(gamepad2.leftStickButtonWasPressed()) {
            spindexer.setBallColors(defaultBallColor);
        };
    }
    private void driveRobot(){
        if(autoDriveToShoot){
            if(followPathCommand != null && followPathCommand.isFinished()){
                autoDriveToShoot = false;
            }
        }
        if(gamepad1.aWasPressed()){
            autoDriveToShoot = true;
            Pose toPose = (shootSide == ShootSide.LEFT) ? toCloseLeftShoot : toCloseRightShoot;
            PathChain pathChain = getPathChain(follower.getPose(), toPose);
            followPathCommand = new FollowPathCommand(follower, pathChain);
            schedule(followPathCommand);
        }
        else if(gamepad1.yWasPressed()){
            if(followPathCommand != null && !followPathCommand.isFinished()) {
                CommandScheduler.getInstance().cancel(followPathCommand);
            }
            autoDriveToShoot = false;
        }

        if(!autoDriveToShoot) {
            follower.setTeleOpDrive(gamepad1.left_stick_y * currSpeed, gamepad1.left_stick_x * currSpeed, turnPower, true);
        }
    }
    private void runGamepad2Commands(){
        flywheelCommands();
        intakeCommands();
        spindexerCommands();
        setBallColorsDefault();

    }

    private void spindexerCommands(){
        //OUTTAKE go to spots
        if(!autoSpindexer && gamepad2.dpadLeftWasPressed() && !autoIntake){
            spindexerGoingToSpot = true;
            goToSpotCurrNum = 0;
            gotToSpotType = SpotType.OUTTAKE;
        }
        else if(!autoSpindexer && gamepad2.dpadDownWasPressed() && !autoIntake){
            spindexerGoingToSpot = true;
           goToSpotCurrNum = 1;
           gotToSpotType = SpotType.OUTTAKE;
        }
        else if(!autoSpindexer && gamepad2.dpadRightWasPressed() && !autoIntake){
            spindexerGoingToSpot = true;
            goToSpotCurrNum = 2;
            gotToSpotType = SpotType.OUTTAKE;
        }
        else if(!autoSpindexer && gamepad2.xWasPressed() && !autoIntake){
            spindexerGoingToSpot = true;
            goToSpotCurrNum = 0;
            gotToSpotType = SpotType.INTAKE;
        }
        else if(!autoSpindexer && gamepad2.aWasPressed() && !autoIntake){
            spindexerGoingToSpot = true;
            goToSpotCurrNum = 1;
            gotToSpotType = SpotType.INTAKE;
        }
        else if(!autoSpindexer && gamepad2.bWasPressed() && !autoIntake){
            spindexerGoingToSpot = true;
            goToSpotCurrNum = 2;
            gotToSpotType = SpotType.INTAKE;
        }


        if(spindexerGoingToSpot){
            spindexerGoingToSpot = false;
            autoSpindexer = true;
            spindexer.getTurner().setRunMode(CRServoEx2.RunMode.OptimizedPositionalControl);
            spindexerGotoSpot = new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(goToSpotCurrNum), gotToSpotType, CRServoEx2.RunMode.OptimizedPositionalControl, 0);
            schedule(spindexerGotoSpot);
        }

        //back to 0 position give back manual control to the driver
        if(autoSpindexer && spindexerGotoSpot != null && spindexerGotoSpot.isFinished() && !autoIntake){
            autoSpindexer = false;
        }

        //manual exit - exit out of auto command


        //manual intake powering
        if(!autoIntake && !autoSpindexer){
//            spindexer.setMode(CRServoEx2.RunMode.RawPower);
            double power = gamepad2.left_stick_y * currturnerSpeed * change;
            spindexerServo.setPower(gamepad2.left_stick_y * currturnerSpeed * change);
            spindexerDirection = power > 0 ? 1: -1;
        }

    }
    private void intakeCommands(){
        if(gamepad2.yWasPressed()){
            autoIntake = true;

            if(spindexer.getNearestEmptyIntakeSpot() == null){
                autoIntake = false;
            }

            if(autoIntake) {
//                goToSpotCommand = new SpindexerGotoSpot(spindexer, spindexer.getNearestEmptyIntakeSpot(), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 0);
//                autoIntakeCommand = new AutoIntakeCommand(spindexer, intake, 1 , 5000);
//                schedule(new SequentialCommandGroup(
//                        goToSpotCommand,
//                        autoIntakeCommand
//                ));
            }

        }

        if(!autoIntake){
            intakePower = gamepad2.right_stick_y * maxIntakePower;
            intake.setDirectPower(intakePower);
        }
    }

    private void flywheelCommands(){
        if(gamepad2.startWasPressed()){
            autoShootSeq = true;
            seqCommand = new ShootSeqCommand(spindexer, shooter, spindexer.getOptimalSequence(pattern), follower, shootSide, false, currentShootDist, true);
        }

        if(autoShootSeq && seqCommand.isFinished()){
            spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
            autoShootSeq = false;
        }
        if(!autoShootSeq) {
            handleShooterInput();
        }
    }
    private void handleOuttakeBallRemoval(){
        boolean shootOn = false;

        if(shooter.getPredictedBotVel() > 50 || shooter.getPredictedTopVel() > 50 || shooter.low.motor.getPower() > 0.05 || shooter.high.motor.getPower() > 0.05){
            shootOn = true;
        }

        SpindexerSpot closestSpot = spindexer.getNearestSpot(spindexer.getCurrentAngle(), SpotType.OUTTAKE);
        if(shootOn && spindexer.getCurrentAngle().sub(closestSpot.getOuttakeAngle()).toDegrees() < shootAngleTolerance){
            spindexer.removeBall(closestSpot.getIndex());
        }
    }

    private void handleShooterInput(){
        if (gamepad2.left_trigger > 0.5) {
            shooterRunMode = shooterRunMode == TwoWheelShooter.RunMode.RawPower ? TwoWheelShooter.RunMode.VelocityControl : TwoWheelShooter.RunMode.RawPower;
            if (shooterRunMode == TwoWheelShooter.RunMode.VelocityControl) {
                resetVelocityGains(shooter, pidBotGainsShooter, kBotGainsShooter, pidTopGainsShooter, kTopGainsShooter);
                shooter.low.setRunMode(Motor.RunMode.VelocityControl);
                shooter.high.setRunMode(Motor.RunMode.VelocityControl);
            }
            shooter.setRunMode(shooterRunMode);
        }


        if (gamepad2.rightBumperWasPressed()) {
            if(useLUT){
                shooter.setFlywheelsPower(follower.getPose(), shootSide);
            }
            else {
                currentShootDist = TwoWheelShooter.ShootDist.Far;
                shooter.setFlywheelsPower(currentShootDist);
            }
        } else if (gamepad2.leftBumperWasPressed()) {
            if(useLUT){
                shooter.setFlywheelsPower(follower.getPose(), shootSide);
            }
            else {
                currentShootDist = TwoWheelShooter.ShootDist.Close;
                shooter.setFlywheelsPower(currentShootDist);
            }
        }
        if (gamepad2.right_trigger > 0.5) {
            shooter.stopFlywheels();
        }
    }
    private void updateTelem() {
        telemetry.addData("Start Pose",  startPose.getPose().toString());
        telemetry.addData("Start Heading(Deg)", "%.4f", convertRadToDegrees(startPose.getHeading()));
        telemetry.addData("Current Pose",  follower.getPose().toString());
        telemetry.addLine("------------------------------------");

        telemetry.addLine("------------------------------------");
        telemetry.addData("Current Speed", currSpeed);
        telemetry.addData("Shoot Side", shootSide);
        telemetry.addData("Current Shoot Dist", currentShootDist);
        telemetry.addData("Pattern", pattern);

        telemetry.addLine("------------------------------------");
        telemetry.addData("Auto Align", autoAlign);
        telemetry.addData("Heading Error(Alignment)", convertRadToDegrees(headingError));
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("Prev Head Weight", alignmentWeight);


        telemetry.addLine("------------------------------------");
        telemetry.addData("Auto Spindexer", autoSpindexer);
        telemetry.addData("Auto Drive",  autoDriveToShoot);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Intake Run Mode", intakeRunMode);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Intake Target Velocity", intakeTargetVel);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Shooter Mode", shooterRunMode);
        telemetry.addData("Shooter Top Power", shooter.high.get());
        telemetry.addData("Shooter Bot Power", shooter.low.get());
        addDataTelemetryGraph("Shooter Top Vel", shooter.high.getVelocity());
        addDataTelemetryGraph("Shooter Bot Vel", shooter.low.getVelocity());
        dashboardTelemetry.addData("Shoter Top Vel", shooter.high.getVelocity());
        dashboardTelemetry.addData("Shoter Bot Vel", shooter.low.getVelocity());
        telemetry.addData("Shooter Bot RunMode", shooter.low.motorEx.getMode());
        telemetry.addData("Shooter Top RunMode", shooter.low.motorEx.getMode());
        addDataTelemetryGraph("Shooter Corr Top Vel", shooter.high.getCorrectedVelocity());
        addDataTelemetryGraph("Shooter Corr Bot Vel", shooter.low.getCorrectedVelocity());
        telemetry.addData("Shooter Dir RunMode", shooter.runMode);
        telemetry.addData("Distance From Goal", shooter.getDistance(follower.getPose(), shootSide));


        telemetry.addLine("--------------------------------");
        telemetry.addData("Spindexer Mode", spindexerRunMode);
        telemetry.addData("Spindexer Angle", spindexer.getCurrentAngle());
        telemetry.addData("Spindexer Norm Angle", spindexer.getEncoder().getAngle());
        telemetry.addData("Spindexer Raw Angle", spindexer.getEncoder().getAngleUnnormalized());
        if(spindexerGotoSpot != null) {
            telemetry.addData("Spindexer GotoSpot Finished", spindexerGotoSpot.isFinished());
        }
        telemetry.addData("Spindexer Going to Spot", spindexerGoingToSpot);
        telemetry.addData("Spindexer Auto Spindxer", autoSpindexer);
        if(spindexer.getSequence() != null) {
            telemetry.addData("Spindexer Seq 0", spindexer.getSequence()[0]);
            telemetry.addData("Spindexer Seq 1", spindexer.getSequence()[1]);
            telemetry.addData("Spindexer Seq 2", spindexer.getSequence()[2]);
        }
        if(spindexer.getBallColors() != null) {
            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
        }
        telemetry.addData("Spindexer Angle Error", spindexer.getTurner().error);
        telemetry.addData("Auto Intake", autoIntake);
        telemetry.addData("Manual Spindexer Power", gamepad2.left_stick_y * currturnerSpeed * change);
        telemetry.addData("Spindexer Direction", spindexerDirection);
        telemetry.addData("Current gotoSpot", goToSpotCurrNum);
        telemetry.addData("Current gotoSpot type", gotToSpotType);
        telemetry.addData("Outake Spindexer Coeff on", spindexer.outakeSpindexerCoeffOn);

        telemetryM.update();;
        graphM.update();
        telemetry.update();
        dashboardTelemetry.update();

    }



}
