package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base;

import static org.firstinspires.ftc.teamcode.main.MainTeleOpTurret.settleTime;

import android.os.Environment;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LambdaCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.commands.Robot;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandTime;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootUpdateCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.newpid.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.GobildaLightBlock;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.subsystems.StopItServo;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter2;
import org.firstinspires.ftc.teamcode.tests.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.AllConfigs;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.AngleNonCR;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ExtraFns;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

@Configurable
public abstract class DanielCloseAutoClean extends CommandOpMode {
    public static Pose shootAtPose1 = new Pose(48, 92);
    public static Pose shootAtPose2 = new Pose(55, 83);
    public static Pose gateIntakePose = new Pose(14, 56.2, Math.toRadians(150));
    public static Pose startPose = new Pose(22, 120, Math.toRadians(142));
    public static double shootTimeout = 700;
    public static double rowXInner = 40;
    public static double rowXOuter = 25;
    public static double row1Y = 38;
    public static double row2Y = 61;
    public static double row3Y = 85;
    public static double headingFacingEdge = -Math.PI;
    PIDController driveController = new PIDController(0.05, 0, 0.001);
    PIDController headingController = new PIDController(1.0, 0, 0.01);

    Follower follower;
    ShootSide side;
//    TelemetryManager telemetryM;
    WheelControl2 drive;
    Timer timer = new Timer(TimeUnit.MILLISECONDS);
    Timer autoElapsed = new Timer(TimeUnit.SECONDS);
    SequentialCommandGroup main;
    Pose robotPose;
    Vector robotVel;
    double currVolt;
    boolean started = false;
    double targetTurretAngle;
    double turretHeadingError;
    double[] aimData;

    enum State {
        init,
        balls3,
        balls6,
        balls9,
        balls12,
        balls15,
        balls18,
        balls21,
        end
    }
    String directoryName = "competition";
    FileWriter xFileWriter;
    FileWriter yFileWriter;
    FileWriter headingFileWriter;
    File xFile;
    File yFile;
    File headingFile;

    String sideFileName = "side.txt";
    String xFileName = "robot_x.txt";
    String yFileName = "robot_y.txt";
    String headingFileName = "robot_heading.txt";
    FileWriter sideFileWriter;
    File sideFile;
    String outputString;
    State state;
    TwoWheelShooter2 shooter;
    Turret turret;
    Intake intake;
    PushUpServo pushUpServo;
    SpindexerNonCR spindexer;
    StopItServo stopItServo;
    VoltageSensor voltageSensor;
    boolean useBulkMode = true;
    Angle wrappedTurretValue;
    double targetHeading;
    public static long totalShootingTime = 450;
    double spindexerSettleTime = 100;
    public abstract ShootSide getShootSide();

    @Override
    public void initialize() {
        super.reset();
        Robot.config = AllConfigs.oldBot;
        side = getShootSide();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        state = State.init;
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        drive = new WheelControl2(hardwareMap)
                .setPidControllers(driveController, headingController);

        follower.setStartingPose(side.fromLeftPose(startPose));

        initializeSubsystems();
        register(intake, shooter, spindexer, pushUpServo, turret);
        if(useBulkMode) {
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

        resetEncoders();
        initCommands();
    }

    public void initializeSubsystems() {
        spindexer = new SpindexerNonCR(hardwareMap,
                true,
                new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);

        shooter = new TwoWheelShooter2(hardwareMap,
                TwoWheelShooter2.RunMode.VelocityControl,
                TwoWheelShooter2.RunMode.VelocityControl);

        pushUpServo = new PushUpServo(hardwareMap, true);
        turret = new Turret(hardwareMap, true);
        stopItServo = new StopItServo(hardwareMap, true);
    }

    public void resetEncoders(){
        //set spindexer to spot 3
        spindexer.setDirectPosition(
                SpindexerSpotNonCR.getPositionFromIndex(3, SpotType.INTAKE) + SpindexerSpotNonCR.OUTAKE_OFFSET_DEGREES / SpindexerNonCR.totalDegrees);
        waitThread(5000);
        //wait for turret to recenter
        turret.resetEncoderPosition();
    }
    public void waitThread(long time){
        try{
            Thread.sleep(time);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void initialize_loop() {
        updateData();
        updateTelemetry();
    }

    @Override
    public void run() {
        super.run();
        updateData();
        if (!started) {
            autoElapsed.restart();
            main.schedule();
            started = true;
        }
        updateTelemetry();
    }

    public void calculateAlign(boolean useSOTM) {
        if(useSOTM){
            double[] aimData;
            aimData = shooter.aimCalculator.targetPowersHeading(
                    follower.getPose(),
                    follower.getVelocity(),
                    TwoWheelShooter2.getShootPoseNew(robotPose, side)
            );
            targetHeading = MathFunctions.normalizeAngle(aimData[2]);
        } else{
            targetHeading = TwoWheelShooter2.getShootHeading(robotPose, side);
        }
        wrappedTurretValue = Angle.fromRadians(ExtraFns.normAnglePlusMinusPI(targetHeading - robotPose.getHeading()));
        turret.setServos(turret.angleToServo(wrappedTurretValue));
        turretHeadingError = turret.getTurretHeadingError(wrappedTurretValue);
    }
    public void setShooterPower(boolean useSOTM){
        //set motors & transfer
        if(useSOTM) {
            shooter.setFlywheelNew(follower.getPose(), follower.getVelocity(), side, currVolt);
        } else{
            shooter.setFlywheelLUT(follower, side, currVolt);
        }
        shooter.setTransferPower(TwoWheelShooter2.transferVelocity, currVolt);
    }

    public void updateData() {
        follower.update();
        robotPose = follower.getPose();
        robotVel = follower.getVelocity();
        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void initCommands() {
        Command balls1To3 = new CommandBase() {
            final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                    () -> ((distToGoal() > 30 && shooter.readyToShoot()
                            && turret.isAtAngle(wrappedTurretValue, false))
                            || timer.getTime() > 3000)
            );
            final Timing.Timer shootTimer = new Timing.Timer(totalShootingTime, TimeUnit.MILLISECONDS);
            double startPosition = spindexer.getCurrentSpindexerPosition();
            double targetPosition =  SpindexerSpotNonCR.OUTAKE_OFFSET_DEGREES / SpindexerNonCR.totalDegrees;
            public void initialize() {
                timer.restart();
                state = State.balls3;
                addRequirements(stopItServo, pushUpServo, shooter, turret);
            }
            boolean start = false;
            public void execute() {
                if(!start){
                    pushUpServo.setUp();
                    stopItServo.setActivePosition();
                    start = true;
                }
                calculateAlign(true);
                setShooterPower(true);
                drive.pidNoHeading(
                        robotPose,
                        new Pose(
                                side.fromLeftX(shootAtPose1.getX()),
                                shootAtPose1.getY()
                        )
                );
                if (shootSupplier.getAsBoolean() && !shootTimer.isTimerOn()) {
                    shootTimer.start();
                }

                //ready to shoot
                if(shootTimer.isTimerOn()){
                    spindexer.setDirectPosition(startPosition + (targetPosition - startPosition) * Math.max(shootTimer.elapsedTime(), totalShootingTime) / totalShootingTime);
                }
            }
            public boolean isFinished() {
                return shootTimer.done();
            }

            public void end(boolean interrupted) {
                spindexer.setDirectPosition(targetPosition);
                stopItServo.setInactivePosition();
                pushUpServo.setDown();
                shooter.stopAll();
                spindexer.setDefault();
            }
        };

        Command balls4To6 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls6),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        side.fromLeftX(rowXInner),
                                        row2Y,
                                        side.fromLeftHeading(headingFacingEdge)
                                ), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() < row2Y + 10),
                // Adjust position and power
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        side.fromLeftX(rowXInner),
                                        row2Y,
                                        side.fromLeftHeading(headingFacingEdge)
                                ), 0.5)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXInner + 1),
                // Drive straight forward and intake
                new ParallelRaceGroup(
                    new LambdaCommand()
                            .setInitialize(() -> timer.restart())
                            .setExecute(() -> drive.pid(
                                    robotPose, new Pose(side.fromLeftX(rowXOuter), row2Y, side.fromLeftHeading(headingFacingEdge)), 0.5)
                            )
                            .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXOuter),
                    new AutoIntakeCommandTime(
                            spindexer,
                            intake,
                            1.0,
                            true,
                            voltageSensor,
                            0,
                            1,
                            spindexerSettleTime
                    )
                ),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXInner), row2Y, side.fromLeftHeading(headingFacingEdge)), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > rowXInner - 14),

                // Drive to shoot & power flywheels at same time
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> {
                                            calculateAlign(false);
                                            drive.pidNoHeading(robotPose, new Pose(
                                                    side.fromLeftX(shootAtPose2.getX()),
                                                    shootAtPose2.getY()
                                            ), 1);
                                        })
                                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                                new LambdaCommand()
                                        .setInitialize(() -> {
                                            timer.restart();
                                            drive.stop();
                                        })
                                        .setIsFinished(() -> timer.getTime() > 500)
                        ),
                        new ShootUpdateCommand(spindexer, shooter, turret, follower, side, voltageSensor, false, 5000)
                ),
                new CommandBase() {
                    final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                            () -> ((ExtraFns.closeZoneDist(robotPose) < 7 && shooter.readyToShoot()
                                    && turret.isAtAngle(wrappedTurretValue, false))
                                    || timer.getTime() > 3000)
                    );
                    final Timing.Timer shootTimer = new Timing.Timer(totalShootingTime, TimeUnit.MILLISECONDS);
                    double startPosition = spindexer.getCurrentSpindexerPosition();
                    double targetPosition =  SpindexerSpotNonCR.OUTAKE_OFFSET_DEGREES / SpindexerNonCR.totalDegrees;
                    public void initialize() {
                        timer.restart();
                        state = State.balls6;
                        addRequirements(stopItServo, pushUpServo, shooter, turret);
                    }
                    boolean start = false;
                    public void execute() {
                        if(!start){
                            pushUpServo.setUp();
                            stopItServo.setActivePosition();
                            start = true;
                        }
                        calculateAlign(false);
                        setShooterPower(false);
                        if (shootSupplier.getAsBoolean() && !shootTimer.isTimerOn()) {
                            shootTimer.start();
                        }

                        //ready to shoot
                        if(shootTimer.isTimerOn()){
                            spindexer.setDirectPosition(startPosition + (targetPosition - startPosition) * Math.max(shootTimer.elapsedTime(), totalShootingTime) / totalShootingTime);
                        }
                    }
                    public boolean isFinished() {
                        return shootTimer.done();
                    }

                    public void end(boolean interrupted) {
                        spindexer.setDirectPosition(targetPosition);
                        stopItServo.setInactivePosition();
                        pushUpServo.setDown();
                        shooter.stopAll();
                        spindexer.setDefault();
                    }
                }


        );

        Function<State, Command> gateIntakeShoot = state -> new SequentialCommandGroup(
                new InstantCommand(() -> this.state = state),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        side.fromLeftX(rowXInner),
                                        gateIntakePose.getY(),
                                        side.fromLeftHeading(gateIntakePose.getHeading())
                                ), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() < gateIntakePose.getY() + 10),
                // Drive to gate
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new LambdaCommand()
                                .setInitialize(() -> timer.restart())
                                .setExecute(() -> drive.pid(robotPose, side.fromLeftPose(gateIntakePose)))
                                .setIsFinished(() -> timer.getTime() > 850 || follower.getVelocity().getMagnitude() < 5),
                        // Wait for gate intake
                        new InstantCommand(() -> drive.stop()),
                        new WaitCommand(700)
                    ),
                    new AutoIntakeCommandTime(
                            spindexer,
                            intake,
                            1.0,
                            true,
                            voltageSensor,
                            0,
                            1,
                            spindexerSettleTime
                    )
                ),

                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pidNoHeading(
                                robotPose, new Pose(side.fromLeftX(rowXInner), row2Y), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > rowXInner - 9),
                // Drive to shoot
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> {
                                            calculateAlign(false);
                                            drive.pidNoHeading(robotPose, new Pose(
                                                    side.fromLeftX(shootAtPose2.getX()),
                                                    shootAtPose2.getY()
                                            ), 1);
                                        })
                                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                                new LambdaCommand()
                                        .setInitialize(() -> {
                                            timer.restart();
                                            drive.stop();
                                        })
                                        .setIsFinished(() -> timer.getTime() > 500)
                        ),
                        new ShootUpdateCommand(spindexer, shooter, turret, follower, side, voltageSensor, false, 5000)
                ),
                new CommandBase() {
                    final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                            () -> ((ExtraFns.closeZoneDist(robotPose) < 7 && shooter.readyToShoot()
                                    && turret.isAtAngle(wrappedTurretValue, false))
                                    || timer.getTime() > 3000)
                    );
                    final Timing.Timer shootTimer = new Timing.Timer(totalShootingTime, TimeUnit.MILLISECONDS);
                    double startPosition = spindexer.getCurrentSpindexerPosition();
                    double targetPosition =  SpindexerSpotNonCR.OUTAKE_OFFSET_DEGREES / SpindexerNonCR.totalDegrees;
                    public void initialize() {
                        timer.restart();
                        addRequirements(stopItServo, pushUpServo, shooter, turret);
                    }
                    boolean start = false;
                    public void execute() {
                        if(!start){
                            pushUpServo.setUp();
                            stopItServo.setActivePosition();
                            start = true;
                        }
                        calculateAlign(false);
                        setShooterPower(false);
                        if (shootSupplier.getAsBoolean() && !shootTimer.isTimerOn()) {
                            shootTimer.start();
                        }

                        //ready to shoot
                        if(shootTimer.isTimerOn()){
                            spindexer.setDirectPosition(startPosition + (targetPosition - startPosition) * Math.max(shootTimer.elapsedTime(), totalShootingTime) / totalShootingTime);
                        }
                    }
                    public boolean isFinished() {
                        return shootTimer.done();
                    }

                    public void end(boolean interrupted) {
                        spindexer.setDirectPosition(targetPosition);
                        stopItServo.setInactivePosition();
                        pushUpServo.setDown();
                        shooter.stopAll();
                        spindexer.setDefault();
                    }
                }

        );

        Command balls7To9 = gateIntakeShoot.apply(State.balls9);
        Command balls10To12 = gateIntakeShoot.apply(State.balls12);
        Command balls13To15 = gateIntakeShoot.apply(State.balls15);
        Command balls16To18 = gateIntakeShoot.apply(State.balls18);

        Command balls19To21 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls21),
                // Drive just before intake
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        side.fromLeftX(rowXInner + 2),
                                        row3Y,
                                        side.fromLeftHeading(headingFacingEdge)
                                ), 0.5)
                        )
                        .setIsFinished(() -> robotPose.getY() < row3Y + 1),
                // Drive straight forward and intake
                new ParallelRaceGroup(
                    new LambdaCommand()
                            .setInitialize(() -> timer.restart())
                            .setExecute(() -> drive.pid(
                                    robotPose, new Pose(
                                            side.fromLeftX(rowXOuter),
                                            row3Y,
                                            side.fromLeftHeading(headingFacingEdge)
                                    ), 0.5)
                            )
                            .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXOuter),
                    new AutoIntakeCommandTime(
                            spindexer,
                            intake,
                            1.0,
                            true,
                            voltageSensor,
                            0,
                            1,
                            spindexerSettleTime
                    )
                ),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXInner), row3Y, side.fromLeftHeading(headingFacingEdge)), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > rowXInner - 14),
                // Drive to shoot
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> {
                                            calculateAlign(false);
                                            drive.pidNoHeading(robotPose, new Pose(
                                                    side.fromLeftX(shootAtPose2.getX()),
                                                    shootAtPose2.getY()
                                            ), 1);
                                        })
                                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                                new LambdaCommand()
                                        .setInitialize(() -> {
                                            timer.restart();
                                            drive.stop();
                                        })
                                        .setIsFinished(() -> timer.getTime() > 500)
                        ),
                        new ShootUpdateCommand(spindexer, shooter, turret, follower, side, voltageSensor, false, 5000)

                ),
                new CommandBase() {
                    final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                            () -> ((ExtraFns.closeZoneDist(robotPose) < 7 && shooter.readyToShoot()
                                    && turret.isAtAngle(wrappedTurretValue, false))
                                    || timer.getTime() > 3000)
                    );
                    final Timing.Timer shootTimer = new Timing.Timer(totalShootingTime, TimeUnit.MILLISECONDS);
                    double startPosition = spindexer.getCurrentSpindexerPosition();
                    double targetPosition =  SpindexerSpotNonCR.OUTAKE_OFFSET_DEGREES / SpindexerNonCR.totalDegrees;
                    public void initialize() {
                        timer.restart();
                        addRequirements(stopItServo, pushUpServo, shooter, turret);
                    }
                    boolean start = false;
                    public void execute() {
                        if(!start){
                            pushUpServo.setUp();
                            stopItServo.setActivePosition();
                            start = true;
                        }
                        calculateAlign(false);
                        setShooterPower(false);
                        if (shootSupplier.getAsBoolean() && !shootTimer.isTimerOn()) {
                            shootTimer.start();
                        }

                        //ready to shoot
                        if(shootTimer.isTimerOn()){
                            spindexer.setDirectPosition(startPosition + (targetPosition - startPosition) * Math.max(shootTimer.elapsedTime(), totalShootingTime) / totalShootingTime);
                        }
                    }
                    public boolean isFinished() {
                        return shootTimer.done();
                    }

                    public void end(boolean interrupted) {
                        spindexer.setDirectPosition(targetPosition);
                        stopItServo.setInactivePosition();
                        pushUpServo.setDown();
                        shooter.stopAll();
                        spindexer.setDefault();
                    }
                }
        );

        Command stop = new InstantCommand(() -> {
            drive.stop();
            state = State.end;
        });

        main = new SequentialCommandGroup(
                balls1To3,
                balls4To6,
                balls7To9,
                balls10To12,
                balls13To15,
                balls16To18,
                balls19To21,
                stop
        );
    }

    public double distToGoal() {
        return TwoWheelShooter.getShootPose(side).distanceFrom(robotPose);
    }

    public void updateTelemetry() {
        telemetry.addData("Auto time elapsed", autoElapsed.getTime());
        telemetry.addData("Timer", timer.getTime());
        telemetry.addData("Robot pose X", robotPose.getX());
        telemetry.addData("Robot pose Y", robotPose.getY());
        telemetry.addData("Robot pose heading", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Robot velocity X", robotVel.getXComponent());
        telemetry.addData("Robot velocity Y", robotVel.getYComponent());

        telemetry.addLine("--------------------");

        telemetry.addData("State", state);
        telemetry.addData("Distance to goal", distToGoal());
    }
    @Override
    public void end(){

        xFile = createFile(xFileName, directoryName);
        yFile = createFile(yFileName, directoryName);
        headingFile = createFile(headingFileName, directoryName);
        try {
            xFileWriter = new FileWriter(xFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            yFileWriter = new FileWriter(yFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            headingFileWriter = new FileWriter(headingFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        follower.update();
        Pose pose = follower.getPose();
        String xLine = String.format("%.4f", pose.getX());
        String yLine = String.format("%.4f", pose.getY());
        String headingLine = String.format("%.4f", pose.getHeading());
        writeToFile(xFileWriter, xLine);
        closeFileWriter(xFileWriter);

        writeToFile(yFileWriter, yLine);
        closeFileWriter(yFileWriter);

        writeToFile(headingFileWriter, headingLine);
        closeFileWriter(headingFileWriter);

        sideFile = createFile(sideFileName, directoryName);
        if(getShootSide() == ShootSide.LEFT){
            outputString = "Left";
        }
        else{
            outputString = "Right";
        }
        try {
            sideFileWriter = new FileWriter(sideFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        //write shoot side
        try {
            sideFileWriter.write(outputString);
            sideFileWriter.flush();
        } catch (IOException e) {
            RobotLog.ee("Log", "No file writer detected: " + e.getMessage());
        }
        //close shoot side
        try {
            sideFileWriter.close();
        } catch (IOException e) {
            RobotLog.ee("Log", "Cannot close file writer: " + e.getMessage());
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

}
