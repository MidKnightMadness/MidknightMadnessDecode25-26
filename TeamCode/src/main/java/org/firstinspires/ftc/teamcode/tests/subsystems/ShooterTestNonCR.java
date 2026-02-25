package org.firstinspires.ftc.teamcode.tests.subsystems;

import static org.firstinspires.ftc.teamcode.util.ExtraFns.normAngle;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.graph.GraphManager;
//import com.bylazar.telemetry.TelemetryManager;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandNonCR;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand2;
import org.firstinspires.ftc.teamcode.commands.spindexer.OutakeSpotsRotation;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPosition;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPositionSmooth;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.GobildaLightBlock;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.robotDrive.WheelControl;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.tests.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.File;
import java.util.Map;

@Config
@TeleOp(name = "Shooter Test NON CR", group = "Competition")
public class ShooterTestNonCR extends CommandOpMode {
    Timer timer;
    SpindexerNonCR spindexer;
    TwoWheelShooter shooter;
    WheelControl wheelControl;
    public static boolean wheelControlUse = true;
    ShootSide shootSide = ShootSide.LEFT;
    boolean autoSpindexer = false;
    boolean autoIntake = false;
    boolean autoShootSeq = false;
    public static double rejectReadingThreshold = 7;
    boolean shootOn;
    boolean readyToShoot = false;
    int spindexerDirection;

    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;
    public static double[] pidAutoAlign = new double[]{1.0, 0, 0.1};//1.5, 0, 0.1
    FollowPathCommand followPathCommand;
    double prevHeadingError = 0;
    double turnPower;
    double headingError;
    public static Intake.RunMode intakeRunMode = Intake.RunMode.RawPower;
    public static CRServoEx2.RunMode spindexerRunMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    SpotType activeSpotType = null;

    public static double change = 0.001;
    public static boolean setCustomPower = false;
    public static double customTopTargetVel = 1700;
    public static double customBotTargetVel = 1400;

    public static double offsetRadians = Math.toRadians(0);
    ShootSeqCommand shootSeqCommand;
    AutoIntakeCommandNonCR autoIntakeCommand;
    SequentialCommandGroup seqAutoIntakeCommand;
    SequentialCommandGroup spindexerGotoPositionSeq;
    SpindexerGotoPosition spindexerGotoPosition;
    SpindexerGotoPositionSmooth spindexerGotoPositionSmooth;
    OutakeSpotsRotation outakeSpotsRotation;

    Telemetry dashboardTelemetry;
    TwoWheelShooter.ShootDist currentShootDist;

    public static boolean useDistanceSensor = true;
    BallColor[] currSpindexerBallColors;
    boolean useLUT = false;
    boolean voltageCompensation = false;
    public static double powerAutoIntake = 0.93;

    Timer gameTimer;

    public static boolean useBulkMode = true;

    PushUpServo pushUpServo;
    public static boolean useDoublePinpoint = false;

    double targetSpindexerPosition;
    int activeSpindexerSpot = 0;
    public static double totalSmoothTime = 1;
    public static double inBetweenOutakeTime = 200;

    @Override
    public void initialize() {
        //TODO: Bulk read testing

        reset();
        timer = new Timer();
        gameTimer = new Timer();

        initializeSubsystems();

        spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        register(shooter, spindexer, pushUpServo);

//
        if(useBulkMode) {
            PhotonCore.disable();
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
            );
        }
        else{
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
            );
        }

        telemetry.setMsTransmissionInterval(500);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    @Override
    public void initialize_loop() {
        telemetry.addData("Spindexer Ball Colors", spindexer.getBallColors());
        telemetry.addData("Spindexer Curr Angle", spindexer.getCurrentAngle());
        telemetry.update();
    }

    public void initializeSubsystems() {
        spindexer = new SpindexerNonCR(hardwareMap, useDistanceSensor, new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
        pushUpServo = new PushUpServo(hardwareMap);


    }

    boolean start;
    @Override
    public void run() {
        if(!start){
            spindexer.getTurnerEncoder().encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.getTurnerEncoder().encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            start = true;
        }
        super.run();
        currSpindexerBallColors = spindexer.getBallColors();


        runGamepad2Commands();

        emergencyStops();

        updateTelem();


        spindexer.updateShootOn(shootOn);

    }



    private void emergencyStops() {
        if (gamepad2.dpadUpWasPressed()) {
            if (autoSpindexer) {
                if (spindexerGotoPosition != null) {
                    CommandScheduler.getInstance().cancel(spindexerGotoPosition);
                }
                resetAutoSpindexer();
                if(spindexerGotoPositionSeq != null){
                    CommandScheduler.getInstance().cancel(spindexerGotoPositionSeq);
                }
                if(outakeSpotsRotation != null){
                    CommandScheduler.getInstance().cancel(outakeSpotsRotation);
                }
            }
        }


    }





    boolean relocalized = false;



    public double smallestAbsDifferenceDegrees(double a, double b) {
        double diff = Math.abs(a - b) % 360.0;
        if (diff > 180) {
            return 360 - diff;
        }
        return diff;
    }


    public static double getDistance(Pose start, Pose target) {
        double dist = Math.sqrt((target.getY() - start.getY()) * (target.getY() - start.getY()) +
                (target.getX() - start.getX()) * (target.getX() - start.getX()));
        return dist;
    }

    public void setBallColorsDefault() {
        if (gamepad2.leftStickButtonWasPressed()) {
            spindexer.setDefault();
        }
    }
    //
    private void runGamepad2Commands() {
        flywheelCommands();
        spindexerCommands();
        pushUpCommands();
        setBallColorsDefault();
    }



    private void pushUpCommands() {
        if(gamepad1.aWasPressed()){
            pushUpServo.setDown();
        }

        if (gamepad2.bWasPressed()) {
            pushUpServo.setUp();
            if (autoSpindexer) {
                resetAutoSpindexer();
            }
        }
    }

    double autoIntakeSpot = 0;
    boolean autoIntakeFinishedReset;

    private void spindexerCommands() {

        if(autoIntake && autoIntakeCommand != null){
            autoIntakeSpot = autoIntakeCommand.getSpotPosition();
        }

        //cycles between spindexer intake spots
        if (gamepad2.dpadLeftWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();
            if(activeSpindexerSpot != 0) {
                activeSpindexerSpot--;
            }
            spindexerGotoPosition = new SpindexerGotoPosition(spindexer, SpindexerSpotNonCR.fromIndex(activeSpindexerSpot).getIntakePositionSolo());
            schedulePosition(spindexerGotoPosition);
        } else if (gamepad2.dpadRightWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();
            if(activeSpindexerSpot != 3) {
                activeSpindexerSpot++;
            }
            spindexerGotoPosition = new SpindexerGotoPosition(spindexer, SpindexerSpotNonCR.fromIndex(activeSpindexerSpot).getIntakePositionSolo());
            schedulePosition(spindexerGotoPosition);
        }

        //finds nearest spindexer intake spot
        else if (gamepad2.dpadDownWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();
            spindexerGotoPosition = new SpindexerGotoPosition(spindexer, spindexer.getNearestIntakePosition(SpotType.INTAKE));
            schedulePosition(spindexerGotoPosition);
        } else if (gamepad2.leftBumperWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();
            spindexerGotoPositionSmooth = new SpindexerGotoPositionSmooth(spindexer, spindexer.startOutakePosition, totalSmoothTime);
            schedulePosition(spindexerGotoPositionSmooth);
        } else if (gamepad2.rightBumperWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();
            spindexerGotoPositionSmooth = new SpindexerGotoPositionSmooth(spindexer, spindexer.endOutakePosition, totalSmoothTime);
            schedulePosition(spindexerGotoPositionSmooth);
        }  else if(gamepad2.yWasPressed()){
            clearExistingSpindexerCommand();;
            outakeSpotsRotation = new OutakeSpotsRotation(spindexer, SpindexerSpotNonCR.SPOT1, -1, inBetweenOutakeTime);
            schedulePosition(outakeSpotsRotation);
        }

    }

    private void clearExistingSpindexerCommand() {
        if (spindexerGotoPositionSeq != null) {
            CommandScheduler.getInstance().cancel(spindexerGotoPositionSeq);
        }
        if(spindexerGotoPosition != null){
            CommandScheduler.getInstance().cancel(spindexerGotoPosition);
        }
        if(outakeSpotsRotation != null){
            CommandScheduler.getInstance().cancel(outakeSpotsRotation);
        }
    }

    private void schedulePosition(Command spindexerGotoPosition) {
        autoSpindexer = true;
        spindexerGotoPositionSeq = new SequentialCommandGroup(spindexerGotoPosition,
                new InstantCommand(() -> resetAutoSpindexer()));
        schedule(spindexerGotoPositionSeq);
    }

    public void resetAutoSpindexer() {
        autoSpindexer = false;
        spindexerGotoPosition = null;
        spindexerGotoPositionSmooth = null;
        spindexerGotoPositionSeq = null;
        targetSpindexerPosition = -1;
    }


    private void flywheelCommands() {

        //shoot sequence command doesn't power the flywheel, need to power using handleShooterInput simaltaenously
        handleShooterInput();

    }

    private void resetAutoShoot() {
        autoShootSeq = false;
        shootSeqCommand = null;
    }

    boolean prevLeftTrigger = false;
    double currVolt;

    private void handleShooterInput() {
        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();


        if(gamepad1.leftBumperWasPressed()){
            setCurrentShootDist(TwoWheelShooter.ShootDist.Close, currVolt);
        } else if(gamepad1.leftBumperWasPressed()){
            setCurrentShootDist(TwoWheelShooter.ShootDist.Far, currVolt);
        }else if (gamepad2.right_trigger > 0.5) {
            shooter.stopFlywheels();
            setBallColorsDefault();
//            shooter.resetGainScheduling();
        }

        if (gamepad2.shareWasPressed()) {
            voltageCompensation = !voltageCompensation;
            if (voltageCompensation) gamepad2.rumbleBlips(2);
        }


    }

    private void setCurrentShootDist(TwoWheelShooter.ShootDist shootDist, double currVolt) {
        currentShootDist = shootDist;
        if (!setCustomPower) {
            shooter.setFlywheelStaticPresets(shootDist, voltageCompensation, currVolt);
        } else {
            shooter.setCustomPower(customBotTargetVel, customTopTargetVel, currVolt);
        }

    }

    //TODO: REORGANIZE TELEMETRY
    private void updateTelem() {


        if (autoIntakeCommand != null && !autoIntakeCommand.isFinished()) {
            telemetry.addData("Curr Target Spot Auto Intake", autoIntakeCommand.currNumSpot);
        }
        telemetry.addData("Relocalized", relocalized);
        if (useDistanceSensor) {
            telemetry.addData("ball detected", spindexer.distCheck);
            telemetry.addData("update rate", 1000.0 / gameTimer.getDeltaTime());


            telemetry.addData("Voltage Use", voltageCompensation);
            telemetry.addData("Use LUT", useLUT);
            telemetry.addData("Shoot Mode", shooterRunMode);
            telemetry.addData("Current Voltage", currVolt);
            telemetry.addLine("------------------------------------");


            telemetry.addLine("------------------------------------");
            telemetry.addData("Shoot Side", shootSide);
            telemetry.addData("Current Shoot Dist", currentShootDist);
            telemetry.addData("Shooter Top Factor", shooter.getCurrTopFactor());
            telemetry.addData("Shooter Bot Factor", shooter.getCurrBotFactor());
            telemetry.addData("Multiplier Top", shooter.getCurrTopFactor() * shooter.getTargetVoltage() / shooter.getCurrVoltage());
            telemetry.addData("Multiplier Bot", shooter.getCurrBotFactor() * shooter.getTargetVoltage() / shooter.getCurrVoltage());

            telemetry.addData("Ready to Shoot", shooter.readyToShoot());
            telemetry.addData("Actual Recovery Time", shooter.getRecoveryTime());

            telemetry.addLine("--------------------------------");
            telemetry.addData("Shooter Mode", shooterRunMode);
            telemetry.addData("Shooter Top Power", shooter.high.get());
            telemetry.addData("Shooter Bot Power", shooter.low.get());
            telemetry.addData("Shooter Top Vel", shooter.high.getVelocity());
            telemetry.addData("Shooter Bot Vel", shooter.low.getVelocity());
            telemetry.addData("Corr Shooter Top", shooter.high.getCorrectedVelocity());
            telemetry.addData("Corr Shooter Bot", shooter.low.getCorrectedVelocity());


            telemetry.addLine("--------------------------------");
            telemetry.addData("Spindexer Mode", spindexerRunMode);
            telemetry.addData("Spindexer Angle", spindexer.getCurrentAngle());
            telemetry.addData("Current Set Position", spindexer.getServo().getPosition());
            telemetry.addData("Curr Active Spot", activeSpindexerSpot);
            telemetry.addData("Auto Intake Spot", autoIntakeSpot);
//            telemetry.addData("Spindexer Auto Spindxer", autoSpindexer);
//            telemetry.addData("Auto Intake Target Spot", targetSpotPosition);
            if (currSpindexerBallColors != null) {
                telemetry.addData("Spindexer Ball Color 0", currSpindexerBallColors[0]);
                telemetry.addData("Spindexer Ball Color 1", currSpindexerBallColors[1]);
                telemetry.addData("Spindexer Ball Color 2", currSpindexerBallColors[2]);
            }
            telemetry.addData("Spindexer Direction", spindexerDirection);

            telemetry.addData("Spindexer Curr Spot Type", activeSpotType);

            telemetry.update();

            dashboardTelemetry.addData("Shoter Top Vel", shooter.high.getVelocity());
            dashboardTelemetry.addData("Shoter Bot Vel", shooter.low.getVelocity());
            dashboardTelemetry.addData("Shooter Corr Top Vel", shooter.high.getCorrectedVelocity());
            dashboardTelemetry.addData("Shooter Corr Bot Vel", shooter.low.getCorrectedVelocity());

            dashboardTelemetry.update();

        }

    }
}
