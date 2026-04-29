package org.firstinspires.ftc.teamcode.main.autonomous.farStart.base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LambdaCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandTime;
import org.firstinspires.ftc.teamcode.commands.spindexer.OuttakeSpotsRotation;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.main.MainTeleOpTurret;
import org.firstinspires.ftc.teamcode.newpid.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.subsystems.StopItServo;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter2;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ExtraFns;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

@Configurable
public abstract class DanielFarAutoReverted extends CommandOpMode {
    //    public static Pose shootAtPose1 = new Pose(48, 92);
    public static Pose shootAtPose2 = new Pose(60, 10);
    public static Pose gateIntakePose = new Pose(3, 60, Math.toRadians(150));
    public static Pose startPose = new Pose(55.5, 8.8, Math.toRadians(90));
    public static Pose endPose = new Pose(45, 34);
    public static double shootTimeout = 700;
    public static double rowXInner = 45;
    public static double rowXOuter = 15;
    public static double row1Y = 38;
    public static double row2Y = 62;
    public static double row3Y = 85;

    public static double headingFacingEdge = -Math.PI;
    PIDController driveController = new PIDController(0.05, 0, 0.001);
    PIDController headingController = new PIDController(1.0, 0, 0.01);

    Follower follower;
    ShootSide side;
    TelemetryManager telemetryM;
    WheelControl2 drive;
    Timer timer = new Timer(TimeUnit.MILLISECONDS);
    Timer autoElapsed = new Timer(TimeUnit.SECONDS);
    SequentialCommandGroup main;
    Pose robotPose;
    Vector robotVel;
    double currVolt;
    boolean started = false;
    double targetHeading;
    double headingError;
    double[] aimData;
    double cornerX;
    double zoneIntakeY = 15;
    double zoneTwoIntakeY = 25;
    double zoneThreeIntakeY = 35;

    enum State {
        init,
        balls3,
        balls6,
        balls9,
        balls12,
        balls15,
        balls18,
        balls21,
        driveToEnd,
        end
    }
    State state;
    TwoWheelShooter2 shooter;
    Turret turret;
    Intake intake;
    double spindexerPosition;
    PushUpServo pushUpServo;
    SpindexerNonCR spindexer;
    StopItServo stopItServo;
    VoltageSensor voltageSensor;
    boolean useBulkMode = true;

    Angle wrappedTurretValue;
    public static long totalShootingTime = 700;
    double spindexerSettleTime = 100;
    double turretHeadingError;


    public abstract ShootSide getShootSide();

    @Override
    public void initialize() {
//        Robot.config = AllConfigs.oldBot;

        super.reset();
        side = getShootSide();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        state = State.init;
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
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


    boolean reset;
    boolean start;

    public void resetEncoders(){
        if(!start){
            timer.restart();
            start = true;
        }
        //set spindexer to spot 3
        spindexer.setDirectPosition(
                SpindexerSpotNonCR.getPositionFromIndex(3, SpotType.INTAKE));

        if(timer.getTime() >= 3000 && !reset){
            turret.resetEncoderPosition();
            reset = true;
        }
        //wait for turret to recenter

        pushUpServo.setUp();
        stopItServo.setActivePosition();
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
        writePose();
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


    public void updateData() {
        follower.update();
        robotPose = follower.getPose();
        robotVel = follower.getVelocity();
        currVolt = voltageSensor.getVoltage();
    }

    public void initCommands() {
        Command balls1To3 = new CommandBase() {
            final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                    () -> ((shooter.readyToShoot() && timer.getTime() > 1000)
                            || timer.getTime() > 3000)
            );
            Timer shootTimer;
            boolean timeStarted;
            OuttakeSpotsRotation command;
            double targetPosition = 0;
            public void initialize() {
                command = new OuttakeSpotsRotation(spindexer, 3, totalShootingTime / 3, totalShootingTime / 2);
                timer.restart();
                shootTimer = new Timer();
                addRequirements(spindexer, shooter, turret);
            }
            public void execute() {

                calculateAlign(true);
                setTransferPower();
                setShooterPower(false);
                if (shootSupplier.getAsBoolean() && !timeStarted) {
                    shootTimer.restart();
                    timeStarted = true;
                }


                //ready to shoot
                if(timeStarted) {
                    command.execute();
                }
            }
            public boolean isFinished() {
                return command.isFinished();
            }

            public void end(boolean interrupted) {
                spindexer.setDirectPosition(targetPosition);
                stopItServo.setInactivePosition();
                spindexer.setDefault();
                pushUpServo.setDown();
                shooter.stopAll();
            }
        };

        Command balls4To6 = new SequentialCommandGroup(
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        side.fromLeftX(rowXInner),
                                        row1Y,
                                        side.fromLeftHeading(headingFacingEdge)
                                ), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() > row1Y - 3),
                // Adjust position and power
                // Drive straight forward and intake
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose, new Pose(
                                                        side.fromLeftX(rowXInner),
                                                        row1Y,
                                                        side.fromLeftHeading(headingFacingEdge)
                                                ), 0.5)
                                        )
                                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXInner + 1),
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose, new Pose(side.fromLeftX(rowXOuter), row1Y, side.fromLeftHeading(headingFacingEdge)), 0.4)
                                        )
                                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXOuter),
                                new InstantCommand(() -> drive.stop()),
                                new WaitCommand(2000)
                        ),
                        new AutoIntakeCommandTime(
                                spindexer,
                                intake,
                                1.0,
                                false,
                                voltageSensor,
                                0,
                                1,
                                spindexerSettleTime
                        )
                ),
                new InstantCommand(()-> resetIntake()),
                new InstantCommand(() -> stopItServo.setActivePosition()),
                new InstantCommand(()-> spindexer.setDirectPosition(SpindexerSpotNonCR.getPositionFromIndex(3, SpotType.INTAKE))),
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            if(timer.getTime() > 300){
                                setTransferPower();
                                pushUpServo.setUp();
                            }
                            calculateAlign(true);
                            setShooterPower(false);
                            drive.pid(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    shootAtPose2.getY(),
                                    targetHeading
                            ), 1);
                        })
                        .setIsFinished(() -> ExtraFns.farZoneDist(robotPose) < 10),

                new InstantCommand(() -> drive.stop()),
                new CommandBase() {
                    final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                            () -> ((shooter.readyToShoot() && timer.getTime() > 500)
                                    || timer.getTime() > 3000)
                    );
                    Timer shootTimer;
                    boolean timeStarted;
                    OuttakeSpotsRotation command;
                    double targetPosition = 0;

                    public void initialize() {
                        command = new OuttakeSpotsRotation(spindexer, 3, totalShootingTime / 3, totalShootingTime / 2);
                        timer.restart();
                        shootTimer = new Timer();
                        addRequirements(spindexer, shooter, turret);
                    }

                    boolean start;

                    public void execute() {

                        calculateAlign(true);
                        setTransferPower();
                        setShooterPower(false);
                        if (shootSupplier.getAsBoolean() && !timeStarted) {
                            shootTimer.restart();
                            timeStarted = true;
                        }


                        //ready to shoot
                        if (timeStarted) {
                            command.execute();
                        }
                    }

                    public boolean isFinished() {
                        return command.isFinished();
                    }

                    public void end(boolean interrupted) {
                        spindexer.setDirectPosition(targetPosition);
                        stopItServo.setInactivePosition();
                        spindexer.setDefault();
                        pushUpServo.setDown();
                        shooter.stopAll();
                    }
                }
        );
        Function<State, Command> cornerIntakeShoot = state -> new SequentialCommandGroup(
                new InstantCommand(() -> this.state = state),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, side.fromLeftPose(new Pose(
                                        rowXInner,
                                        zoneIntakeY,
                                        headingFacingEdge
                                )), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXInner),
                // Drive to corner
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose,
                                                side.fromLeftPose(new Pose(0, zoneIntakeY, headingFacingEdge)),
                                                0.4
                                        ))
                                        .setEnd(() -> cornerX = side.fromLeftX(robotPose.getX()))
                                        .setIsFinished(() -> timer.getTime() > 1200 && follower.getVelocity().getMagnitude() < 2),
                                // Drive straight back
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose,
                                                side.fromLeftPose(new Pose(cornerX + 5, zoneIntakeY, headingFacingEdge)),
                                                1
                                        ))
                                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > cornerX + 5),
                                // Drive into wall
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose,
                                                side.fromLeftPose(new Pose(cornerX + 5, 0, headingFacingEdge)),
                                                1
                                        ))
                                        .setEnd(() -> zoneIntakeY = robotPose.getY() + 7)
                                        .setIsFinished(() -> timer.getTime() > 300 && Math.abs(robotVel.getYComponent()) < 2),
                                // Drive forward again
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose,
                                                side.fromLeftPose(new Pose(-30, 0, headingFacingEdge)),
                                                0.4
                                        ))
                                        .setIsFinished(() -> timer.getTime() > 300 && follower.getVelocity().getMagnitude() < 2),
                                new InstantCommand(()-> drive.stop()),
                                new WaitCommand(1400)
                        ),
                        new AutoIntakeCommandTime(
                                spindexer,
                                intake,
                                1.0,
                                false,
                                voltageSensor,
                                0,
                                1,
                                spindexerSettleTime
                        )
                ),
                new InstantCommand(() -> stopItServo.setActivePosition()),
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign(true);
                            setShooterPower(false);
                            if(timer.getTime() > 300) {
                                pushUpServo.setUp();
                                setTransferPower();
                            }
                            drive.pidNoHeading(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    zoneIntakeY + 8
                            ), 1);
                        })
                        .setIsFinished(() -> ExtraFns.farZoneDist(robotPose) < 7),

                new InstantCommand(()-> drive.stop()),
                new CommandBase() {
                    final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                            () -> ((shooter.readyToShoot() && timer.getTime() > 500)
                                    || timer.getTime() > 3000)
                    );
                    Timer shootTimer;
                    boolean timeStarted;
                    OuttakeSpotsRotation command;
                    double targetPosition = 0;

                    public void initialize() {
                        command = new OuttakeSpotsRotation(spindexer, 3, totalShootingTime / 3, totalShootingTime / 2);
                        timer.restart();
                        shootTimer = new Timer();
                        addRequirements(spindexer, shooter, turret);
                    }

                    boolean start;

                    public void execute() {

                        calculateAlign(true);
                        setTransferPower();
                        setShooterPower(false);
                        if (shootSupplier.getAsBoolean() && !timeStarted) {
                            shootTimer.restart();
                            timeStarted = true;
                        }


                        //ready to shoot
                        if (timeStarted) {
                            command.execute();
                        }
                    }

                    public boolean isFinished() {
                        return command.isFinished();
                    }

                    public void end(boolean interrupted) {
                        spindexer.setDirectPosition(targetPosition);
                        stopItServo.setInactivePosition();
                        spindexer.setDefault();
                        pushUpServo.setDown();
                        shooter.stopAll();
                    }
                }
        );
        Function<State, Command> secondaryIntakeShoot = state -> new SequentialCommandGroup(
                new InstantCommand(() -> this.state = state),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, side.fromLeftPose(new Pose(
                                        rowXInner,
                                        zoneTwoIntakeY,
                                        headingFacingEdge
                                )), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXInner),
                // Drive to corner
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose,
                                                side.fromLeftPose(new Pose(0, zoneTwoIntakeY, headingFacingEdge)),
                                                0.4
                                        ))
                                        .setEnd(() -> cornerX = side.fromLeftX(robotPose.getX()))
                                        .setIsFinished(() -> timer.getTime() > 850 || follower.getVelocity().getMagnitude() < 5),
                                // Drive straight back
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose,
                                                side.fromLeftPose(new Pose(cornerX + 5, zoneTwoIntakeY, headingFacingEdge)),
                                                1
                                        ))
                                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > cornerX + 5),
                                // Drive forward again
                                new LambdaCommand()
                                        .setInitialize(() -> timer.restart())
                                        .setExecute(() -> drive.pid(
                                                robotPose,
                                                side.fromLeftPose(new Pose(-30, zoneThreeIntakeY, headingFacingEdge)),
                                                0.4
                                        ))
                                        .setIsFinished(() -> timer.getTime() > 300 && follower.getVelocity().getMagnitude() < 5),
                                new InstantCommand(()-> drive.stop()),
                                new WaitCommand(1400)
                        ),
                        new AutoIntakeCommandTime(
                                spindexer,
                                intake,
                                1.0,
                                false,
                                voltageSensor,
                                0,
                                1,
                                spindexerSettleTime
                        )
                ),
                new InstantCommand(() -> stopItServo.setActivePosition()),
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign(true);
                            setShooterPower(false);
                            if(timer.getTime() > 300) {
                                pushUpServo.setUp();
                                setTransferPower();
                            }
                            drive.pidNoHeading(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    zoneIntakeY + 8
                            ), 1);
                        })
                        .setIsFinished(() -> ExtraFns.farZoneDist(robotPose) < 7),

                new InstantCommand(()-> drive.stop()),
                new CommandBase() {
                    final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                            () -> ((shooter.readyToShoot() && timer.getTime() > 500)
                                    || timer.getTime() > 3000)
                    );
                    Timer shootTimer;
                    boolean timeStarted;
                    OuttakeSpotsRotation command;
                    double targetPosition = 0;
                    public void initialize() {
                        command = new OuttakeSpotsRotation(spindexer, 3, totalShootingTime / 3, totalShootingTime / 2);
                        timer.restart();
                        shootTimer = new Timer();
                        addRequirements(spindexer, shooter, turret);
                    }
                    boolean start;
                    public void execute() {

                        calculateAlign(true);
                        setTransferPower();
                        setShooterPower(false);
                        if (shootSupplier.getAsBoolean() && !timeStarted) {
                            shootTimer.restart();
                            timeStarted = true;
                        }


                        //ready to shoot
                        if(timeStarted) {
                            command.execute();
                        }
                    }
                    public boolean isFinished() {
                        return command.isFinished();
                    }

                    public void end(boolean interrupted) {
                        spindexer.setDirectPosition(targetPosition);
                        stopItServo.setInactivePosition();
                        spindexer.setDefault();
                        pushUpServo.setDown();
                        shooter.stopAll();
                    }
                }
        );

        Command balls7to9 = cornerIntakeShoot.apply(State.balls9);
        Command balls9to12 = secondaryIntakeShoot.apply(State.balls12);
        Command balls12to15 = secondaryIntakeShoot.apply(State.balls15);
        Command balls16to18 = secondaryIntakeShoot.apply(State.balls18);

        Command driveToEnd = new LambdaCommand()
                .setInitialize(() -> {
                    state = State.driveToEnd;
                    timer.restart();
                })
                .setExecute(() -> drive.pidNoHeading(robotPose, side.fromLeftPose(endPose)))
                .setIsFinished(() -> robotPose.getY() > endPose.getY() - 5);

        Command stop = new InstantCommand(() -> {
            drive.stop();
            state = State.end;
        });

        main = new SequentialCommandGroup(
                balls1To3,
                balls4To6,
                balls7to9,
                balls9to12,
                balls12to15,
                balls16to18,
                driveToEnd,
                stop
        );
    }
    public void resetIntake(){
        intake.setDirectPower(0);
    }

    public void setShooterPower(boolean useSOTM){
        //set motors & transfer
        if(useSOTM) {
            shooter.setFlywheelNew(follower.getPose(), follower.getVelocity(), side, currVolt);
        } else{
            shooter.setFlywheelLUT(follower, side, currVolt);
        }
    }
    public void setTransferPower(){
        shooter.setTransferPower(TwoWheelShooter2.transferVelocity, currVolt);
    }


    public double distToGoal() {
        return TwoWheelShooter2.getShootPose(side).distanceFrom(robotPose);
    }
//    @Override
//    public void end(){
//        ConstantsBot.robotPose = robotPose;
//        ConstantsBot.side = side;
//    }


    public void writePose() {
        MainTeleOpTurret.startPoseX = robotPose.getX();
        MainTeleOpTurret.startPoseY  = robotPose.getY();
        MainTeleOpTurret.startPoseHeading = robotPose.getHeading();

        ConstantsBot.side = side;
    }
    Telemetry dashboardTelemetry;
    public void updateTelemetry() {
        telemetry.addData("Update Rate", 1000.0/ timer.getDeltaTime());
        telemetry.addData("Turret Heading Error", turretHeadingError);;
        telemetry.addData("Auto time elapsed", autoElapsed.getTime());
        telemetry.addData("Timer", timer.getTime());
        telemetry.addData("Robot pose X", robotPose.getX());
        telemetry.addData("Robot pose Y", robotPose.getY());
        telemetry.addData("Robot pose heading", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Robot velocity X", robotVel.getXComponent());
        telemetry.addData("Robot velocity Y", robotVel.getYComponent());
        telemetry.addData("Low Error", shooter.bottomError);
        telemetry.addData("High Error", shooter.topError);
        telemetry.addData("Transfer Error", (shooter.transferError));

        telemetry.addLine("--------------------");

        dashboardTelemetry.addData("Low Error", shooter.bottomError);
        dashboardTelemetry.addData("High Error", shooter.topError);
        dashboardTelemetry.addData("Transfer Error", (shooter.transferError));
        telemetry.addData("State", state);
        telemetry.addData("Distance to goal", distToGoal());
        telemetry.addData("Spindexer position", spindexerPosition);

        dashboardTelemetry.update();
        telemetry.update();
    }
}