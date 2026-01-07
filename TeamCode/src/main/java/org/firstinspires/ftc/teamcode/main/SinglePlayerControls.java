package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.util.ExtraFns.normAngle;

import android.os.Environment;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoAngle;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
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
@TeleOp(name = "Single Player", group = "Competition")
public class SinglePlayerControls extends CommandOpMode {
    Follower follower;
    Pose startPose = new Pose(72, 8, Math.toRadians(90));
    Pose currentPose;
    Timer timer;

    Spindexer spindexer;
    Ramp ramp;
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
    File motifFile;
    String id;

    double currSpeed = 0.8;
    double maxSpeed = 0.8;
    double midSpeed = 0.5;
    public static double intakePower = 1;
    double maxIntakePower = 1;
    double intakeTargetVel = 3;

    public static double[] pidGainsShooter = new double[]{0.01, 0, 0.001};
    public static double[] kGainsShooter = new double[]{0, 0, 0};
    public static double[] pidIntakeGains = new double[]{0.01, 0, 0};
    public static double[] kIntakeGains = new double[]{0, 0, 0};
    double topShooterPower = 0.8;
    double botShooterPower = 0.6;
    Pose toCloseLeftShoot = new Pose(72, 72, Math.toRadians(135));
    Pose toCloseRightShoot = new Pose(72, 72, Math.toRadians(135));
    Pose toFarLeftShoot = new Pose(67, 11, Math.toRadians(110));
    Pose toFarRightShoot = new Pose(5, 11, Math.toRadians(70));
    ShootSide shootSide = ShootSide.LEFT;
    public static double headingErrorMax = 0.3;
    public static double pathDistThresholdMax = 3;
    CRServo spindexerServo;

    double currturnerSpeed = 1;
    double maxTurnerSpeed = 1;
    double change = 1;

    Pose leftTarget = new Pose(0, 144, Math.toRadians(45));
    Pose rightTarget = new Pose(144, 144, Math.toRadians(-45));
    ShootHardcode spindexerHardcode;

    boolean autoAlign = false;
    boolean autoSpindexer = false;
    boolean autoDriveToShoot = false;

    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.RawPower;

    public static double[] pidAutoAlign = new double[]{1.5, 0, 0.1};
    public static double[] alignmentWeights = new double[]{0.2, 0.8};
    FollowPathCommand followPathCommand;
    double prevHeadingError = 0;
    double turnPower;
    double headingError;
    public static Intake.RunMode intakeRunMode = Intake.RunMode.RawPower;
    public static CRServoEx2.RunMode spindexerRunMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    public static double spindexerFinishedTimeThreshold = 300;//mms
    //    SpindexerGotoSpot spindexerSpotCommand;
    SpindexerGotoAngle spindexerGoToAngle;

    public static double[] spindexerIntakeAngles = new double[]{70, 140, 210};
    int currentIntakeSpot = 0;
    boolean intakePowerOn = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );

        ConstantsBot.motifIsBusy = false;
        timer = new Timer();
        spindexer = new Spindexer(hardwareMap);
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.RawPower);

        pattern = readMotifFromFile(motifFileName);
        double robotX = readDoubleFromPose(botXFileName);
        double robotY = readDoubleFromPose(botYFileName);
        double robotHeading = readDoubleFromPose(botHeadingFileName);
        shootSide = readShootSideFromFile(sideFileName);

        Pose roboPose = new Pose(robotX, robotY, robotHeading);
        startPose = roboPose != null ? roboPose : startPose;


        follower = ConstantsBot.createPinpointFollowerCustom(hardwareMap, startPose);
        intake = new Intake(hardwareMap, intakeRunMode);
        if(intakeRunMode == Intake.RunMode.VelocityControl){
            intake.setPid(pidIntakeGains[0], pidIntakeGains[1], pidIntakeGains[2]);
            intake.setFeedforward(kIntakeGains[0], kIntakeGains[1], kIntakeGains[2]);
        }

        if(shooterRunMode == TwoWheelShooter.RunMode.VelocityControl) {
            shooter.setPid(pidGainsShooter[0], pidGainsShooter[1], pidGainsShooter[2]);
            shooter.setFeedforward(kGainsShooter[0], kGainsShooter[1], kGainsShooter[2]);
        }

        follower.startTeleopDrive();
        spindexerServo = spindexer.getTurner().getServo();
//        spindexerHardcode = new ShootHardcode(spindexer, shooter, pattern, true);
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
            return fileName.equals("Left") ? ShootSide.LEFT : ShootSide.RIGHT;
        }
        else {
            return ShootSide.LEFT;
        }
    }


    @Override
    public void run(){
        super.run();
        runGamepad1Comands();

        follower.update();

        updateTelem();


    }

    public PathChain getPathChain(Pose pose1, Pose pose2){
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(pose1.getPose(), pose2.getPose()))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .setHeadingConstraint(headingErrorMax)
                .setTimeoutConstraint(3000)
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


    private double calculateGamepadPID(double prevHeadingError, double headingError){//alignment(headingwise) to pose
        double filteredHeadingError = alignmentWeights[1] * headingError + alignmentWeights[0] * prevHeadingError;
        double power = pidAutoAlign[0] * filteredHeadingError + pidAutoAlign[2] * (filteredHeadingError - prevHeadingError) * timer.getDeltaTime();
        //ignore i for now
        power = (power <= 1 &&  power>=-1) ? power : power > 1 ? 1 : -1;
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
        headingError = getAngleError(outakeSide, ((shootSide == ShootSide.LEFT) ? leftTarget : rightTarget));
        if(autoAlign) {
            turnPower = calculateGamepadPID(prevHeadingError, headingError);
        }
        prevHeadingError = headingError;
    }
    private void runGamepad1Comands(){
        currentPose = follower.getPose();

        setAlignTurnPower();
        driveRobot();//includes automations
        manualChangeShootSide();
        manualChangeMotif();
        flywheelCommands();
        intakeCommands();
        spindexerCommands();
        //change min/max speed

        //x, y, b, a
        //sticks
        //left t, right t, right b
        //dpadleft

        if (gamepad1.yWasPressed()) {
            currSpeed = currSpeed == maxSpeed ? midSpeed : maxSpeed;
        }

        //change autoAlignment to the goal
        if(gamepad1.xWasPressed()){
            autoAlign = !autoAlign;
        }

    }
    private void runGamepad2Commands(){

    }

    private void manualChangeShootSide(){
        if(gamepad1.aWasPressed()){
            shootSide = shootSide == ShootSide.LEFT ? ShootSide.RIGHT : ShootSide.LEFT;
        }
    }
    //manual changing of motif pattern - in case was not detected
    private void manualChangeMotif(){
        if(gamepad1.dpadLeftWasPressed()){
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
    private void driveRobot(){
//        if(autoDriveToShoot){
//            if(followPathCommand.isFinished()){
//                autoDriveToShoot = false;
//            }
//        }
//        if(gamepad1.aWasPressed()){
//            autoDriveToShoot = true;
//            Pose toPose = (shootSide == ShootSide.LEFT) ? toCloseLeftShoot : toCloseRightShoot;
//            PathChain pathChain = getPathChain(follower.getPose(), toPose);
//            followPathCommand = new FollowPathCommand(follower, pathChain);
//            schedule(followPathCommand);
//        }
//        else if(gamepad1.xWasPressed()){
//            autoDriveToShoot = true;
//            Pose toPose = (shootSide == ShootSide.LEFT) ? toFarLeftShoot : toFarRightShoot;
//            PathChain pathChain = getPathChain(follower.getPose(), toPose);
//            followPathCommand = new FollowPathCommand(follower, pathChain);
//            schedule(followPathCommand);
//        }
        if(!autoDriveToShoot) {
            follower.setTeleOpDrive(gamepad1.left_stick_y * currSpeed, gamepad1.left_stick_x * currSpeed, turnPower, true);
        }
    }


    private void spindexerCommands(){
        if(gamepad1.dpadDownWasPressed()){//go through all the intake spots
            //always go to the next spot -> for intaking
            autoSpindexer = true;
            spindexerGoToAngle = new SpindexerGotoAngle(spindexer, new Angle(spindexerIntakeAngles[currentIntakeSpot], AngleUnit.DEGREES), CRServoEx2.RunMode.OptimizedPositionalControl);
            schedule(spindexerGoToAngle);
            currentIntakeSpot = (currentIntakeSpot + 1) % 3;
        }

        //back to 0 position give back manual control to the driver
        if(currentIntakeSpot == 0 && autoSpindexer && spindexerGoToAngle.isFinished()){
            autoSpindexer = false;
        }

        if(gamepad1.dpadUpWasPressed()){
            autoSpindexer = !autoSpindexer;
            if(!spindexerGoToAngle.isFinished()) {
                spindexerGoToAngle.end(true);
            }
        }

        if(!autoSpindexer){
            double dir = gamepad1.left_bumper ? 1 : -1;
            spindexerServo.setPower(gamepad1.left_trigger * currturnerSpeed * dir * change);
        }
    }
    private void intakeCommands(){
        if(gamepad1.bWasPressed()){
            intakePowerOn = !intakePowerOn;
        }
        intake.setDirectPower(intakePowerOn ? intakePower : 0);
    }

    private void flywheelCommands(){
        topShooterPower = shooter.high.get();
        botShooterPower = shooter.low.get();

        if(gamepad1.rightBumperWasPressed()){
            shooter.setFlywheelStaticPresets(TwoWheelShooter.ShootDist.Close, true);
        }
        else if(gamepad1.right_trigger >= 0.5){
            shooter.stopFlywheels();
        }
        if(gamepad1.right_trigger > 0.2 && gamepad1.right_trigger < 0.5){
            shooter.setFlywheelStaticPresets(TwoWheelShooter.ShootDist.Far, true);
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
        telemetry.addData("Pattern", pattern);

        telemetry.addLine("------------------------------------");
        telemetry.addData("Auto Align", autoAlign);
        telemetry.addData("Heading Error(Alignment)", convertRadToDegrees(headingError));
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("Prev Head Weight", alignmentWeights[0]);
        telemetry.addData("Curr Head Weight", alignmentWeights[1]);


        telemetry.addLine("------------------------------------");
        telemetry.addData("Auto Spindexer", autoSpindexer);
        telemetry.addData("Auto Drive",  autoDriveToShoot);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Intake Run Mode", intakeRunMode);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Intake Power On", intakePowerOn);
        telemetry.addData("Intake Target Velocity", intakeTargetVel);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Shooter Mode", shooterRunMode);
        telemetry.addData("Shooter Top Power", topShooterPower);
        telemetry.addData("Shooter Bot Power", botShooterPower);
        telemetry.addData("Shooter Top Vel", shooter.high.getCorrectedVelocity());
        telemetry.addData("Shooter Bot Vel", shooter.low.getCorrectedVelocity());

        telemetry.addLine("--------------------------------");
        telemetry.addData("Spindexer Mode", spindexerRunMode);
        telemetry.addData("Spindexer Angle", spindexer.getCurrentAngle());
        telemetry.addData("Spindexer Intake Spot", currentIntakeSpot);

        telemetry.update();

    }
    public void addStringToTelem(String s, String o){
        telemetry.addLine(s + o);
    }

    public void addToTelemGraph(String s, Number o){
        telemetry.addData(s, o);
    }



}
