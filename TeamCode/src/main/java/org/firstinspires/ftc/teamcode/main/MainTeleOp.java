package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.util.ExtraFns.normAngle;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand2;
import org.firstinspires.ftc.teamcode.commands.misc.ServoController;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.lights.GobildaLightBlock;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.tests.opModes.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    Timer arducamTimer;

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
    public static boolean recoveryOn = false;

    double maxSpeed = 1.0;
    double midSpeed = 0.5;
    double currSpeed = maxSpeed;
    double intakePower = 1.0;
    double maxIntakePower = 1.0;
    double intakeTargetVel = 3;
    public static double minHeadingError = Math.toRadians(1);
    TelemetryManager telemetryM;
    GraphManager graphM;

    public static double autoIntakePower = 1;
    public static double[] pidIntakeGains = new double[]{0.0004, 0.0, 0.00001};
    public static double[] kIntakeGains = new double[]{0, 0, 0};
    double topShooterPower = 0.8;
    double botShooterPower = 0.6;
    WheelControl wheelControl;
    public static boolean wheelControlUse = true;
    Pose toCloseLeftShoot = new Pose(57, 94, Math.toRadians(310));
    Pose toCloseRightShoot =  new Pose(87, 94, Math.toRadians(230));
    Pose toFarLeftShoot = new Pose(67, 17, Math.toRadians(300));
    Pose toFarRightShoot = new Pose(84, 17, Math.toRadians(240));
    Pose gateIntakeLeft = new Pose(12, 60, Math.toRadians(150));
    Pose gateIntakeRight = new Pose(132, 60, Math.toRadians(30));
    Pose parkRight = new Pose(104, 33, Math.toRadians(90));
    Pose parkLeft = new Pose(144-104, 33, Math.toRadians(90));
    ShootSide shootSide = ShootSide.LEFT;
    double rightAprilAngle = 38.565 + 180;//degrees
    double leftAprilAngle = 360 - 38.565;
    CRServo spindexerServo;
//    CRServo spindexerServo2;


    double maxTurnerSpeed = 1;
    public static double currturnerSpeed = 0.3;//0.3 before
    double midTurnerSpeed = 0.5;

    Pose leftTarget = new Pose(4, 144, Math.toRadians(45));
    Pose rightTarget = new Pose(140, 144, Math.toRadians(-45));


    boolean autoAlign = false;
    public static boolean useArducam = true;
    boolean autoSpindexer = false;
    boolean autoDriveToShoot = false;
    boolean autoIntake = false;
    boolean autoShootSeq = false;
    boolean triggerArducamDetection;

    public static double rejectReadingThreshold = 7;
    boolean shootOn;
    boolean readyToShoot = false;
    public static double spindexerCompensationOffset = Math.toRadians(5);//degrees
    int spindexerDirection;

    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;
//    VelocityControl

    public static double[] pidAutoAlign = new double[]{1.0, 0, 0.1};//1.5, 0, 0.1
    public static double alignmentWeight = 0.2;
    FollowPathCommand followPathCommand;
    double prevHeadingError = 0;
    double turnPower;
    double headingError;
    public static double swapToCameraThreshold = Math.toRadians(20);
    double minPowerHeadingAlign = 0.01;
    public static Intake.RunMode intakeRunMode = Intake.RunMode.RawPower;
    public static CRServoEx2.RunMode spindexerRunMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    //    public static double spindexerFinishedTimeThreshold = 300;//mms
//    SpindexerGotoSpot spindexerSpotCommand;
    SpindexerGotoSpot spindexerGotoSpot;
    int activeSpindexerSpotIndex = -1;
    //    int requestedSpindexerSpotIndex = -1;
    SpotType activeSpotType = null;
//    SpotType requestedSpotType = null;

    public static double change = 1;
    public static boolean setCustomPower = false;
    public static double customTopTargetVel = 1700;
    public static double customBotTargetVel = 1400;
    public static double customTopPower = 0;
    public static double customBotPower = 0;

    public static double offsetRadians = Math.toRadians(0);

    int currentIntakeSpot = 0;
    public static double shootAngleTolerance = 10;
    ShootSeqCommand shootSeqCommand;
    SpindexerGotoSpot goToSpotCommand;
    AutoIntakeCommand2 autoIntakeCommand;
    SequentialCommandGroup seqAutoIntakeCommand;
    boolean mapDistToShoot = true;

    Telemetry dashboardTelemetry;
    TwoWheelShooter.ShootDist currentShootDist;

    public static boolean useDistanceSensor = true;

    public static double pathDistThresholdMax = 0;
    public static double headingErrorMax = 0;
    public static double timeOutConstraint = 0;
    double targetheading = 0;


    boolean hasRumbledAllOccupied = false;
    int goToSpotIntakeNum = 0;
    public static boolean readPoseFile = true;//true
    boolean triggerBallShot = false;
    int recentTriggeredSpot = -1;
    BallColor[] defaultBallColor = new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE};
    BallColor[] currSpindexerBallColors;
    boolean useLUT = false;
    boolean voltageCompensation = false;

    public static double powerAutoIntake = 0.8;

    Pose[] leftGateBounds = new Pose[]{new Pose(14, 52, 0), new Pose(45, 85, 0)};
    Pose[] rightGateBounds = new Pose[]{new Pose(99, 52, 0), new Pose(130, 85, 0)};

    GobildaLightBlock[] spindexerLights;
    GobildaLightBlock pushUpLight;
    GobildaLightBlock readyToShootLight;
    Timer gameTimer;
    int triggeredSpot = -1;
    double slowTurn = 0.6;
    AprilTagWebcam arducam;
    double cameraYawRelative = 0;
    double cameraYawGlobal = 0;
    boolean velAgressiveComp = false;
    public static boolean useBulkMode = true;
    boolean startedBulkMode = false;

    PushUpServo pushUpServo;
    public static boolean useDoublePinpoint = false;
    double spindexerRawPower;
    AprilTagDetection tag;
    BallColor spin1Color;
    BallColor spin2Color;
    BallColor spin3Color;
    GobildaLightBlock.Color readyToShootColor;
    GobildaLightBlock.Color pushUpColor;
    double prevArducamTime = 0;
    public static double minArduTimeUpdate = 0;
    public static double autoSettleTime = 200;
    public static long autoIntakeTimeout = 10000;
    public static Pose failsafeLeftPose = new Pose(8.85, 8, Math.toRadians(180));
    public static Pose failsafeRightPose = new Pose(144-8.85, 8, Math.toRadians(0));

    @Override
    public void initialize() {
        //TODO: Bulk read testing

        reset();

//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        graphM = PanelsGraph.INSTANCE.getManager();

//       ConstantsBot.motifIsBusy = false; -> meant for pinpointAprilTagLocalizer
        timer = new Timer();
//        arducamTimer = new Timer();

        if(readPoseFile) {
            pattern = readMotifFromFile(motifFileName);
            double robotX = readDoubleFromPose(botXFileName);
            double robotY = readDoubleFromPose(botYFileName);
            double robotHeading = readDoubleFromPose(botHeadingFileName);

            shootSide = readShootSideFromFile(sideFileName);
            startPose = new Pose(robotX, robotY, robotHeading);
        }

        gameTimer = new Timer();
//
//        currentShootDist = (startPose.getY() > 20) ? TwoWheelShooter.ShootDist.Close : TwoWheelShooter.ShootDist.Far;

        if(!useDoublePinpoint) {
            follower = ConstantsBot.createPinpointFollower(hardwareMap);
        } else{
            follower = ConstantsBot.createDoublePinpointFollower(hardwareMap);
        }
        follower.setPose(startPose);
        currentPose = startPose;

        initializeSubsystems();


        if(wheelControlUse){
            wheelControl = new WheelControl(hardwareMap);
        }
        else{
            follower.startTeleopDrive();
        }

        spindexerServo = spindexer.getTurner().getServo();
//        spindexerServo2 = spindexer.getTurner2().getServo();

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
        goToSpotCommand = new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, spindexerRunMode, 0);
        spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});


//        updateStartShootDist();
        register(intake, shooter, spindexer, pushUpServo);

        telemetry.setMsTransmissionInterval(250);
    }

    @Override
    public void initialize_loop(){
        telemetry.addData("Follower Pose", follower.getPose());
        telemetry.addData("Spindexer Ball Colors", spindexer.getBallColors());
        telemetry.addData("Spindexer Curr Angle", spindexer.getCurrentAngle());
        telemetry.update();
    }

    public void initializeSubsystems(){
        spindexer = new Spindexer(hardwareMap, useDistanceSensor, new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        spindexer.setMode(spindexerRunMode);
        intake = new Intake(hardwareMap, intakeRunMode);
        if(intakeRunMode == Intake.RunMode.VelocityControl){
            resetVelocityGains(intake, pidIntakeGains, kIntakeGains);
        }

        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
//        shooter.resetEncoders();

        spindexerLights = new GobildaLightBlock[3];
        spindexerLights[0] = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.spindexerLights1));
        spindexerLights[1] = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.spindexerLights2));
        spindexerLights[2] = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.spindexerLights3));

        pushUpLight = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.light4));
        readyToShootLight = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.light5));

        pushUpServo = new PushUpServo(hardwareMap);
        if(useArducam) {
            arducam = new AprilTagWebcam();
            arducam.init(hardwareMap, ConfigNames.arducam, telemetry);
        }
        if(useBulkMode) {
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
            );
        } else{
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
            );
        }

    }


    public void resetVelocityGains(Intake intake, double[] pidIntakeGains, double[] kIntakeGains){
        intake.setPid(pidIntakeGains[0], pidIntakeGains[1], pidIntakeGains[2]);
        intake.setFeedforward(kIntakeGains[0], kIntakeGains[1], kIntakeGains[2]);
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
            currentShootDist = (currentPose.getY() > 20) ? TwoWheelShooter.ShootDist.Close : TwoWheelShooter.ShootDist.Far;
        }
    }
    boolean gameTimerStart = false;

    boolean rumbledLastFive = false;

    @Override
    public void run(){
        super.run();
        currSpindexerBallColors = spindexer.getBallColors();

        if(!gameTimerStart){
            gameTimerStart = true;
            gameTimer.restart();
        }
        if(!rumbledLastFive && gameTimer.getTime() >= 115000){//endgame
            gamepad1.rumble(3000);
            gamepad2.rumble(3000);
            rumbledLastFive = true;
        }

        updateLights();
//
//        if((autoIntake || autoSpindexer) && activeSpindexerSpotIndex != -1 && activeSpotType != null){
//            spindexer.goToSpot(SpindexerSpot.fromIndex(activeSpindexerSpotIndex), activeSpotType, spindexerRunMode);
//        }

        runGamepad1Comands();
        runGamepad2Commands();
        if(gamepad1.aWasPressed()){
            calibrateYaw();
        }
        emergencyStops();


        if(gamepad2.touchpadWasPressed()){
            currturnerSpeed = currturnerSpeed == maxTurnerSpeed ? midTurnerSpeed : maxTurnerSpeed;
        }



//        currSpindexerBallColors = spindexer.getBallColors();

        //only triggers shot if spindexer in spot and bottom flywheel on

        updateTelem();

        if(velAgressiveComp && !shooter.inRecoveryMode){
            velAgressiveComp = false;
        }


        if(pushUpColor == GobildaLightBlock.Color.GREEN){
            shootOn = true;
        } else{
            shootOn = false;
        }

        spindexer.updateShootOn(shootOn);

        if(!shootOn || currSpindexerBallColors == null){
            return;
        }



        if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT0, SpotType.OUTTAKE)){
            triggeredSpot = 0;
            velAgressiveComp = true;
        } else if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT1, SpotType.OUTTAKE)){
            triggeredSpot = 1;
            velAgressiveComp = true;
        } else if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT2, SpotType.OUTTAKE)){
            triggeredSpot = 2;
            velAgressiveComp = true;
        } else{
            triggeredSpot = -1;
            triggerBallShot = false;
        }

        if(!triggerBallShot && triggeredSpot != -1){
            recentTriggeredSpot = triggeredSpot;
            shooter.triggerBallShot(recoveryOn);
//            spindexer.removeBall(triggeredSpot);
            triggerBallShot = true;
        }

    }


    private void updateLights(){
        if(currSpindexerBallColors != null) {
            if(spin1Color != currSpindexerBallColors[0]) {
                spindexerLights[0].setColor(currSpindexerBallColors[0]);
                spin1Color = currSpindexerBallColors[0];
            }
            if(spin2Color != currSpindexerBallColors[1]) {
                spindexerLights[1].setColor(currSpindexerBallColors[1]);
                spin2Color = currSpindexerBallColors[1];
            }
            if(spin3Color != currSpindexerBallColors[2]) {
                spindexerLights[2].setColor(currSpindexerBallColors[2]);
                spin3Color = currSpindexerBallColors[2];
            }
        }


        if(readyToShootLight != null) {
            GobildaLightBlock.Color targetReadyToShoot = headingError <= Math.toRadians(1) ? GobildaLightBlock.Color.GREEN : GobildaLightBlock.Color.ORANGE;
            if(targetReadyToShoot != readyToShootColor) {
                readyToShootLight.setColor(targetReadyToShoot);
                readyToShootColor = targetReadyToShoot;
            }
        }

        if(pushUpLight != null){
            GobildaLightBlock.Color targetPushUpColor = Math.abs(pushUpServo.getServo().getPosition()- pushUpServo.getMinPos()) < 0.1 ? GobildaLightBlock.Color.ORANGE : GobildaLightBlock.Color.GREEN;
            if(targetPushUpColor != pushUpColor){
                pushUpLight.setColor(targetPushUpColor);
                pushUpColor = targetPushUpColor;
            }
        }


    }
    //rumble to notify both gamepads that all balls are occupied
    private void rumbleAllOccuppiedBalls(){
        if(spindexer.allOccuppiedBallColors() && !hasRumbledAllOccupied){
            gamepad2.rumble(500);
            gamepad1.rumble(500);
            hasRumbledAllOccupied = true;
        } else if(!spindexer.allOccuppiedBallColors()){
            hasRumbledAllOccupied = false;
        }
    }


    private void emergencyStops() {
        if(gamepad2.dpadUpWasPressed()) {
            if (autoSpindexer) {
                resetAutoSpindexer();
//                if (spindexerGotoSpot != null) {
//                    CommandScheduler.getInstance().cancel(spindexerGotoSpot);
//                }
//                spindexerServo.setPower(0);
//                spindexerServo2.setPower(0);
            }

            if (autoIntake) {
                if (seqAutoIntakeCommand != null) {
                    CommandScheduler.getInstance().cancel(seqAutoIntakeCommand);
                }
                resetAutoIntake();
            }

            if (autoShootSeq) {
                if(shootSeqCommand != null){
                    CommandScheduler.getInstance().cancel(shootSeqCommand);
                }
                resetAutoShoot();
            }
        }

        if (gamepad1.yWasPressed()) {
            if (autoDriveToShoot) {
                if (followPathCommand != null) {
                    CommandScheduler.getInstance().cancel(followPathCommand);
                }
            }
            follower.breakFollowing();
            if(!wheelControlUse) {
                follower.startTeleopDrive();
            }
            followPathCommand = null;
            autoDriveToShoot = false;
        }
    }


    public PathChain getPathChain(Pose pose1, Pose pose2){
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(pose1.getPose(), pose2.getPose()))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
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

    private double normAnglePlusMinusPI(double error){
        while(error < -Math.PI){
            error += Math.PI *2;
        }
        while(error > Math.PI){
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


//          if(Math.abs(power) <= minPowerHeadingAlign) {
//            power = 0;
//        }

        if (Math.abs(filteredHeadingError) > Math.toRadians(1.0)) {
            power += Math.signum(filteredHeadingError) * 0.1;
        }
        else{
            power = 0;
        }


        return power;
    }
    public double getAngleError(Pose position, Pose target, double positionHeading){
        double deltaY = target.getY() - position.getY();
        double deltaX = target.getX() - position.getX();
        double heading = Math.atan2(deltaY, deltaX);
        heading = normAngle(heading);
        this.targetheading = heading;
        //heading is in absolute degrees
        double error = heading - positionHeading;
        double errorSign = (error > 0 ) ? -1 : 1;
        if(Math.abs(error) > Math.PI){
            error = errorSign * (2 * Math.PI - Math.abs(positionHeading - heading));
        }

        error = normAnglePlusMinusPI(error);
        return error;
    }
    private double convertRadToDegrees(double val){
        return val * 180 / Math.PI;
    }
    boolean swappedToArducam = false;

    private void setAlignTurnPower() {
        turnPower = 0;//REMOVE?
        //MODIFY so that the heading is facing the outake side, not the intake side
        if (autoAlign) {

            Pose outakePose = new Pose(currentPose.getX(), currentPose.getY(), normAngle(Math.toRadians(currentPose.getHeading()) + Math.PI));
            //add compensation for spindexer direction
//            double distToTarget = getDistance(follower.getPose(), outakePose);
//            double compY = outakePose.getY() + distToTarget * Math.tan(spindexerCompensationOffset);
            // double compY = outakePose.getY() + spindexerDirection * distToTarget * Math.tan(spindexerCompensationOffset);
//            Pose compensatedPose = new Pose(outakePose.getX(), outakePose.getY(), outakePose.getHeading());
//            if (spindexer.getTurner().getServo().getPower() < 0.1) {
//                compensatedPose = outakePose;
//            }

//            if (useArducam && arducamTimer.getTime() - prevArducamTime >= minArduTimeUpdate) {{
            headingError = getAngleError(outakePose, ((shootSide == ShootSide.LEFT) ? leftTarget : rightTarget), outakePose.getHeading());


            turnPower = calculateGamepadPID(prevHeadingError, headingError);
            prevHeadingError = headingError;
        }
    }

    boolean relocalized = false;
    private void calibrateYaw() {
        if (useArducam) {
            arducam.update();
            if (shootSide == ShootSide.LEFT) {
                tag = arducam.getTagBySpecificId(20);
            } else {
                tag = arducam.getTagBySpecificId(24);
            }
            if (tag != null) {
                cameraYawRelative = -tag.ftcPose.pitch;
                cameraYawGlobal = (cameraYawRelative + ((shootSide == ShootSide.LEFT) ? leftAprilAngle : rightAprilAngle));//degrees
                if (smallestAbsDifferenceDegrees(cameraYawGlobal, currentPose.getHeading()) < rejectReadingThreshold) {
                    follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(cameraYawGlobal)));
                }
                relocalized = true;
            }
        }
    }
        public double smallestAbsDifferenceDegrees(double a, double b) {
            double diff = Math.abs(a - b) % 360.0;
            if (diff > 180) {
                return 360 - diff;
            }
            return diff;
        }


        public static double getDistance (Pose start, Pose target){
            double dist = Math.sqrt((target.getY() - start.getY()) * (target.getY() - start.getY()) +
                    (target.getX() - start.getX()) * (target.getX() - start.getX()));
            return dist;
        }


        private void runGamepad1Comands(){
            currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));

            manualResetPose();
            setAlignTurnPower();
            driveRobot();//includes automations
            manualChangeShootSide();
            manualChangeMotif();
            speedChange();
            toggleAutoAlign();
            rumbleCloseToGate();

        }

        private void manualResetPose(){
            if(gamepad1.left_trigger > 0.5){
                follower.setPose(failsafeLeftPose);
            } else if(gamepad1.right_trigger > 0.5){
                follower.setPose(failsafeRightPose);
            }
        }
        private void rumbleCloseToGate () {
            Pose[] check = null;
            if (shootSide == ShootSide.LEFT) {
                check = leftGateBounds;
            } else {
                check = rightGateBounds;
            }

            if (currentPose.getX() >= check[0].getX() && currentPose.getY() >= check[0].getY()
                    && currentPose.getX() <= check[1].getX() && currentPose.getY() <= check[1].getY()) {
                gamepad1.rumbleBlips(2);//not sure if blips is best method here
            }
        }

        private void speedChange () {
            if (gamepad1.rightBumperWasPressed()) {
                currSpeed = currSpeed == maxSpeed ? midSpeed : maxSpeed;
            }
        }

        private void toggleAutoAlign () {
            if (gamepad1.leftBumperWasPressed()) {
                autoAlign = !autoAlign;
                prevHeadingError = 0;
            }
        }

        private void manualChangeShootSide () {
            if (gamepad1.xWasPressed()) {
                shootSide = shootSide == ShootSide.LEFT ? ShootSide.RIGHT : ShootSide.LEFT;
            }
        }
        //manual changing of motif pattern - in case was not detected
        private void manualChangeMotif () {
            if (gamepad1.bWasPressed()) {
                if (pattern == MotifEnums.Motif.NONE) {
                    pattern = MotifEnums.Motif.PGP;
                } else if (pattern == MotifEnums.Motif.PGP) {
                    pattern = MotifEnums.Motif.PPG;
                } else if (pattern == MotifEnums.Motif.PPG) {
                    pattern = MotifEnums.Motif.GPP;
                } else {
                    pattern = MotifEnums.Motif.NONE;
                }
            }
        }

        public void setBallColorsDefault () {
            if (gamepad2.leftStickButtonWasPressed()) {
                spindexer.setDefault();
            }
        }
        private void driveRobot () {
//        if (autoDriveToShoot) {
//            if (followPathCommand != null && followPathCommand.isFinished()) {
//                autoDriveToShoot = false;
//            }
//        }

            if (gamepad1.dpadUpWasPressed()) {
                if (followPathCommand != null && !followPathCommand.isFinished()) {
                    CommandScheduler.getInstance().cancel(followPathCommand);
                }
                autoDriveToShoot = true;
                Pose toPose = (shootSide == ShootSide.LEFT) ? toFarLeftShoot : toFarRightShoot;
                PathChain pathChain = getPathChain(currentPose, toPose);
                followPathCommand = new FollowPathCommand(follower, pathChain).setGlobalMaxPower(1.0);
                schedule(followPathCommand);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                if (followPathCommand != null && !followPathCommand.isFinished()) {
                    CommandScheduler.getInstance().cancel(followPathCommand);
                }
                autoDriveToShoot = true;
                Pose toPose = (shootSide == ShootSide.LEFT) ? gateIntakeLeft : gateIntakeRight;
                PathChain pathChain = getPathChain(currentPose, toPose);
                followPathCommand = new FollowPathCommand(follower, pathChain).setGlobalMaxPower(1.0);
                schedule(followPathCommand);
            }

            if (gamepad1.dpadDownWasPressed()) {
                if (followPathCommand != null && !followPathCommand.isFinished()) {
                    CommandScheduler.getInstance().cancel(followPathCommand);
                }
                autoDriveToShoot = true;
                Pose toPose = (shootSide == ShootSide.LEFT) ? toCloseLeftShoot : toCloseRightShoot;
                PathChain pathChain = getPathChain(currentPose, toPose);
                followPathCommand = new FollowPathCommand(follower, pathChain).setGlobalMaxPower(1.0);
                schedule(followPathCommand);
            }
            if (gamepad1.dpadRightWasPressed()) {
                if (followPathCommand != null && !followPathCommand.isFinished()) {
                    CommandScheduler.getInstance().cancel(followPathCommand);
                }
                autoDriveToShoot = true;
                Pose toPose = (shootSide == ShootSide.LEFT) ? parkRight : parkLeft;//reversed
                PathChain pathChain = getPathChain(currentPose, toPose);
                followPathCommand = new FollowPathCommand(follower, pathChain).setGlobalMaxPower(1.0);
                schedule(followPathCommand);
            }


            if (!autoDriveToShoot) {
                if (!wheelControlUse) {
                    if (!autoAlign) {
                        follower.setTeleOpDrive(gamepad1.left_stick_y * currSpeed, gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, true);
                    } else {
                        follower.setTeleOpDrive(gamepad1.left_stick_y * currSpeed, gamepad1.left_stick_x * currSpeed, turnPower, true);
                    }
                } else {
                    if (!autoAlign) {
                        wheelControl.drive_relative(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x * currSpeed, currSpeed);
                    } else {
                        wheelControl.drive_relative(gamepad1.left_stick_y, gamepad1.left_stick_x, turnPower, currSpeed);
                    }
                }
            }

            follower.update();
        }
        private void runGamepad2Commands () {
            flywheelCommands();
            intakeCommands();
            spindexerCommands();
            pushUpCommands();
            setBallColorsDefault();
//        rumbleAllOccuppiedBalls();
            rumbleReadyToShoot();
        }

        boolean hasRumbledreadyToShoot = false;
        boolean hasRumbledAligned = false;
        boolean aligned = false;

        private void rumbleReadyToShoot () {
            //aligned and right velocity
            readyToShoot = shooter.readyToShoot();
            aligned = autoAlign;
//            if (readyToShoot) {
//                gamepad2.rumble(100);
//                gamepad1.rumble(100);
//            }
//            if (!shootOn) {
//                hasRumbledreadyToShoot = false;
//            }

            if (readyToShoot && headingError <= Math.toRadians(1) && aligned) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
//                hasRumbledAligned = true;
            }
//            } else if (turnPower > 0.1) {
//                hasRumbledAligned = false;
//            }

        }

        private void pushUpCommands () {
            if (activeSpindexerSpotIndex !=-1 || (autoIntake && autoIntakeCommand.currNumSpot != -1) || (gamepad2.aWasPressed() && pushUpColor != GobildaLightBlock.Color.ORANGE)) {
                pushUpServo.setDown();
            }
//            if(gamepad1.aWasPressed()){
//                pushUpServo.setDown();
//            }

            if (gamepad2.bWasPressed()) {
                pushUpServo.setUp();
                if (autoSpindexer) {
                    resetAutoSpindexer();
                }
            }
        }
        private void spindexerCommands () {
            if (gamepad2.dpadLeftWasPressed() && !autoIntake) {
                activeSpindexerSpotIndex = (goToSpotIntakeNum + 1) % 3;
                goToSpotIntakeNum = activeSpindexerSpotIndex;
                activeSpotType = SpotType.INTAKE;
                autoSpindexer = true;
            } else if (gamepad2.dpadDownWasPressed() && !autoIntake) {
                activeSpindexerSpotIndex = spindexer.getNearestSpot(spindexer.getCurrentAngle(), SpotType.INTAKE).getIndex();
                goToSpotIntakeNum = activeSpindexerSpotIndex;
                activeSpotType = SpotType.INTAKE;
                autoSpindexer = true;
            } else if (gamepad2.dpadRightWasPressed() && !autoIntake) {
                activeSpindexerSpotIndex = (goToSpotIntakeNum - 1) % 3;
                if(activeSpindexerSpotIndex < 0) activeSpindexerSpotIndex += 3;
                goToSpotIntakeNum = activeSpindexerSpotIndex;
                activeSpotType = SpotType.INTAKE;
                autoSpindexer = true;
            }

            if(autoIntake && autoIntakeCommand != null){
                activeSpindexerSpotIndex = autoIntakeCommand.getSpotCurrent();
            }

            if ((autoIntake && autoIntakeCommand != null) ||(autoSpindexer && activeSpotType != null && activeSpindexerSpotIndex != -1)) {
                spindexer.goToSpot(SpindexerSpot.fromIndex(activeSpindexerSpotIndex), activeSpotType, CRServoEx2.RunMode.OptimizedPositionalControl);
            }

            else if (!autoSpindexer && !autoIntake) {
                spindexerRawPower = gamepad2.left_stick_y * currturnerSpeed * change;
                spindexerServo.setPower(spindexerRawPower);
                spindexerDirection = spindexerRawPower > 0 ? 1 : -1;
            }


        }

        public void resetAutoSpindexer () {
            autoSpindexer = false;
            spindexerGotoSpot = null;
            activeSpindexerSpotIndex = -1;
        }
        private void intakeCommands () {
            if (gamepad2.xWasPressed()) {
                autoIntake = true;
                autoSpindexer = false;
                activeSpindexerSpotIndex = -1;

                autoIntakeCommand = new AutoIntakeCommand2(spindexer, intake, powerAutoIntake, autoSettleTime, useDistanceSensor, hardwareMap);

//                if (seqAutoIntakeCommand != null && !seqAutoIntakeCommand.isFinished()) {
//                    CommandScheduler.getInstance().cancel(seqAutoIntakeCommand);
//                }

                seqAutoIntakeCommand = new SequentialCommandGroup(
                        autoIntakeCommand.withTimeout(autoIntakeTimeout),
                        new InstantCommand(() -> resetAutoIntake())
                );

                schedule(seqAutoIntakeCommand);
            }


            if (!autoIntake) {
                intakePower = gamepad2.right_stick_y * maxIntakePower;
                intake.setDirectPower(gamepad2.right_stick_y * maxIntakePower, currVolt);
            }
        }

        private void resetAutoIntake () {
            autoIntake = false;
            seqAutoIntakeCommand = null;
            activeSpindexerSpotIndex = -1;
            autoIntakeCommand = null;
            intake.setDirectPower(0);
        }

        private void flywheelCommands () {
//        if(gamepad2.bWasPressed()){
//            autoShootSeq = true;
//            if(shootSeqCommand != null && !shootSeqCommand.isFinished()){
//                CommandScheduler.getInstance().cancel(shootSeqCommand);
//            }
//            shootSeqCommand = new ShootSeqCommand(spindexer, shooter, spindexer.getOptimalSequence(pattern), follower, shootSide, false, currentShootDist, false);
//            schedule(shootSeqCommand);
//        }
//
//        if(autoShootSeq && shootSeqCommand != null && shootSeqCommand.isFinished()){
//            resetAutoShoot();
//        }

            //shoot sequence command doesn't power the flywheel, need to power using handleShooterInput simaltaenously
            handleShooterInput();

            //not rlly necesarry
//        if(gamepad2.backWasPressed()){
//            shooter.resetRecoveryFactors();
//        }
        }
        private void resetAutoShoot () {
            autoShootSeq = false;
            shootSeqCommand = null;
        }

        boolean prevLeftTrigger = false;
        double currVolt;
        private void handleShooterInput () {
            currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();


            if (gamepad2.left_bumper) {
                useLUT = false;
                setCurrentShootDist(TwoWheelShooter.ShootDist.Close, currVolt);
            } else if (gamepad2.right_bumper) {
                useLUT = false;
                setCurrentShootDist(TwoWheelShooter.ShootDist.Far, currVolt);
            }  else if (gamepad2.left_trigger > 0.5) {
                useLUT = true;
                setCurrentShootDist(TwoWheelShooter.ShootDist.Far, currVolt);
            }


            if (gamepad2.optionsWasPressed()) {
                shooterRunMode = shooterRunMode == TwoWheelShooter.RunMode.RawPower ? TwoWheelShooter.RunMode.VelocityControl : TwoWheelShooter.RunMode.RawPower;
                shooter.setRunMode(shooterRunMode);
            }
            if (gamepad2.shareWasPressed()) {
                voltageCompensation = !voltageCompensation;
                if (voltageCompensation) gamepad2.rumbleBlips(2);
            }

            if (gamepad2.right_trigger > 0.5) {
                shooter.stopFlywheels();
            }
        }

        private void setCurrentShootDist (TwoWheelShooter.ShootDist shootDist, double currVolt){
            currentShootDist = shootDist;
            if (!setCustomPower) {
                if (useLUT) {
                    shooter.setFlywheelLUT(follower, shootSide, voltageCompensation, currVolt);
                } else {
//                shooter.resetDefaultGains();
                    shooter.setFlywheelPresets(shootDist, follower, shootSide, voltageCompensation, currVolt);
                }
            } else {
                shooter.setCustomPower(customBotTargetVel, customTopTargetVel, currVolt);
            }

        }
        //TODO: REORGANIZE TELEMETRY
        private void updateTelem () {

        if(autoIntakeCommand != null && !autoIntakeCommand.isFinished()) {
            telemetry.addData("Curr Target Spot Auto Intake", autoIntakeCommand.currNumSpot);
        }
            telemetry.addData("Relocalized", relocalized);
            if (useDistanceSensor) {
                telemetry.addData("ball detected", spindexer.distCheck);
//        telemetry.addData("dist 2", spindexer.distanceSensor2.getDistance(DistanceUnit.INCH));
            }
            // telemetry.addData("Near Wheel", spindexer.nearWheel);
            // telemetry.addData("Min Power Overcome", spindexer.getTurner().getMinPowerOvercome());
//        if(spindexer.getNearestEmptyIntakeSpot() != null) {
//            telemetry.addData("Nearest Empty Intake Spot", spindexer.getNearestEmptyIntakeSpot().getIndex());
//        }
            //telemetry.addData("Distance Sensor", spindexer.getDistance());
            //telemetry.addData("Spindexer Closest Intake Spot", spindexer.getNearestSpot(spindexer.getCurrentAngle(), SpotType.INTAKE));
            telemetry.addData("update rate", 1000.0 / gameTimer.getDeltaTime());


            telemetry.addData("follower velocity X,Y,T", "%f , %f, %f", follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), follower.getVelocity().getTheta());

            telemetry.addData("Voltage Use", voltageCompensation);
            telemetry.addData("Use LUT", useLUT);
            telemetry.addData("Shoot Mode", shooterRunMode);
            telemetry.addData("Current Voltage", currVolt);
           // telemetry.addData("Ratio Voltage ", shooter.getTargetVoltage() / shooter.getCurrVoltage());
            telemetry.addData("Start Pose", startPose.getPose().toString());
            telemetry.addData("Start Heading(Deg)", "%.4f", convertRadToDegrees(startPose.getHeading()));
            telemetry.addLine("Current Pose" +  currentPose.getX() + "; " + currentPose.getY() + "; " + currentPose.getHeading());
//        telemetry.addData("Ball Color Sensor 1", spindexer.getBallColor1());
//        telemetry.addData("Ball Color Sensor 2", spindexer.getBallColor2());
            telemetry.addLine("------------------------------------");


            telemetry.addLine("------------------------------------");
            telemetry.addData("Current Speed", currSpeed);
            telemetry.addData("Shoot Side", shootSide);
            telemetry.addData("Current Shoot Dist", currentShootDist);
            // telemetry.addData("Pattern", pattern);
            telemetry.addData("Shooter Top Factor", shooter.getCurrTopFactor());
            telemetry.addData("Shooter Bot Factor", shooter.getCurrBotFactor());
            telemetry.addData("Multiplier Top", shooter.getCurrTopFactor() * shooter.getTargetVoltage() / shooter.getCurrVoltage());
            telemetry.addData("Multiplier Bot", shooter.getCurrBotFactor() * shooter.getTargetVoltage() / shooter.getCurrVoltage());
            telemetry.addData("Trigger Ball Shot", triggerBallShot);
            telemetry.addData("Recently Triggered Shot", recentTriggeredSpot);
            telemetry.addData("Ready to Shoot", shooter.readyToShoot());
            telemetry.addData("Actual Recovery Time", shooter.getRecoveryTime());


            telemetry.addLine("------------------------------------");
            telemetry.addData("Auto Align", autoAlign);
            telemetry.addData("Target Heading", convertRadToDegrees(targetheading));
            telemetry.addData("Heading Error(Alignment)", convertRadToDegrees(headingError));
            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Camera Yaw Global", cameraYawGlobal);
            telemetry.addData("Camera Yaw Rel", cameraYawRelative);
            telemetry.addData("Tag", tag == null ? "NONE" : tag.id);


            telemetry.addLine("------------------------------------");
            telemetry.addData("Auto Spindexer", autoSpindexer);
            telemetry.addData("Auto Drive", autoDriveToShoot);

            telemetry.addLine("--------------------------------");
            telemetry.addData("Intake Run Mode", intakeRunMode);
            telemetry.addData("Intake Power", intakePower);
//        telemetry.addData("Intake Target Velocity", intakeTargetVel);

            telemetry.addLine("--------------------------------");
            telemetry.addData("Shooter Mode", shooterRunMode);
            telemetry.addData("Shooter Top Power", shooter.high.get());
            telemetry.addData("Shooter Bot Power", shooter.low.get());
            telemetry.addData("Shooter Top Vel", shooter.high.getVelocity());
            telemetry.addData("Shooter Bot Vel", shooter.low.getVelocity());
            telemetry.addData("Corr Shooter Top", shooter.high.getCorrectedVelocity());
            telemetry.addData("Corr Shooter Bot", shooter.low.getCorrectedVelocity());

//        dashboardTelemetry.addData("Spindexer Req Spot Indx", requestedSpindexerSpotIndex);
//        dashboardTelemetry.addData("Spindexer Req Spot Type", requestedSpotType);
//        dashboardTelemetry.addData("Spindexer Active Spot Indx", activeSpindexerSpotIndex);
//        dashboardTelemetry.addData("Spindexer Active Spot Type", activeSpotType);

//        dashboardTelemetry.addData("Shoter Top Vel", shooter.high.getVelocity());
//        dashboardTelemetry.addData("Shoter Bot Vel", shooter.low.getVelocity());
//////        telemetry.addData("Shooter Bot RunMode", shooter.low.motorEx.getMode());
//////        telemetry.addData("Shooter Top RunMode", shooter.low.motorEx.getMode());
//        dashboardTelemetry.addData("Shooter Corr Top Vel", shooter.high.getCorrectedVelocity());
//        dashboardTelemetry.addData("Shooter Corr Bot Vel", shooter.low.getCorrectedVelocity());
//        telemetry.addData("Shooter Dir RunMode", shooter.runMode);
            telemetry.addData("Distance From Goal", shooter.getDistance(currentPose, shootSide));


            telemetry.addLine("--------------------------------");
            telemetry.addData("Spindexer Mode", spindexerRunMode);
            telemetry.addData("Spindexer Angle", spindexer.getCurrentAngle());
//        telemetry.addData("Spindexer Norm Angle", spindexer.getEncoder().getAngle());
//        telemetry.addData("Spindexer Raw Angle", spindexer.getEncoder().getAngleUnnormalized());
            telemetry.addData("Spindexer Auto Spindxer", autoSpindexer);
//        if(spindexer.getSequence() != null) {
//            telemetry.addData("Spindexer Seq 0", spindexer.getSequence()[0]);
//            telemetry.addData("Spindexer Seq 1", spindexer.getSequence()[1]);
//            telemetry.addData("Spindexer Seq 2", spindexer.getSequence()[2]);
//        }
        if(currSpindexerBallColors != null) {
            telemetry.addData("Spindexer Ball Color 0", currSpindexerBallColors[0]);
            telemetry.addData("Spindexer Ball Color 1", currSpindexerBallColors[1]);
            telemetry.addData("Spindexer Ball Color 2", currSpindexerBallColors[2]);
        }
            telemetry.addData("Spindexer Angle Error", spindexer.getTurner().error);
            //telemetry.addData("Auto Intake", autoIntake);
            // telemetry.addData("Manual Spindexer Power", gamepad2.left_stick_y * currturnerSpeed * change);
            telemetry.addData("Spindexer Direction", spindexerDirection);
            telemetry.addData("Spindexer Curr Active Indx", activeSpindexerSpotIndex);
//        telemetry.addData("Spindexer Req Indx", requestedSpindexerSpotIndex);

            telemetry.addData("Spindexer Raw Power", spindexerRawPower);
            telemetry.addData("Spindexer Curr Spot Type", activeSpotType);
//        telemetry.addData("Spindexer Req Spot Type", requestedSpotType);
            // telemetry.addData("Outake Spindexer Coeff on", spindexer.outakeSpindexerCoeffOn);

//        telemetryM.update();;
//        graphM.update();
            telemetry.update();
//        dashboardTelemetry.update();

        }

    }
