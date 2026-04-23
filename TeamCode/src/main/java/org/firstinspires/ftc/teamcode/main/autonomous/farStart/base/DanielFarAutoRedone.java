package org.firstinspires.ftc.teamcode.main.autonomous.farStart.base;

import android.os.Environment;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
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
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandTime;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
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

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

@Configurable
public abstract class DanielFarAutoRedone extends CommandOpMode {
    //    public static Pose shootAtPose1 = new Pose(48, 92);
    public static Pose shootAtPose2 = new Pose(60, 10);
    public static Pose gateIntakePose = new Pose(3, 60, Math.toRadians(150));
    public static Pose startPose = new Pose(55.5, 8.8, Math.toRadians(90));
    public static Pose endPose = new Pose(45, 34);
    public static double shootTimeout = 700;
    public static double rowXInner = 40;
    public static double rowXOuter = 20;
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
    public static long totalShootingTime = 480;
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
//            PhotonCore.disable();
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
        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void initCommands() {
        Command balls1To3 = new CommandBase() {
            final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                    () -> ((shooter.readyToShoot() && timer.getTime() > 1000)
                            || timer.getTime() > 3000)
            );
            Timer shootTimer;
            boolean timeStarted;
            double startPosition = (SpindexerSpotNonCR.getPositionFromIndex(3, SpotType.INTAKE));
            double targetPosition = 0;
            public void initialize() {
                timer.restart();
                shootTimer = new Timer();
                addRequirements(stopItServo, pushUpServo, shooter, turret);
            }
            boolean start;
            public void execute() {
                if(!start){
                    start = true;
                }
                calculateAlign(true);
                setTransferPower();
                if (shootSupplier.getAsBoolean() && !timeStarted) {
                    shootTimer = new Timer();
                    timeStarted = true;
                }


                //ready to shoot
                if(timeStarted) {
                    spindexerPosition = startPosition + (targetPosition - startPosition) * Math.max(shootTimer.getTime(), totalShootingTime) / totalShootingTime;
                    spindexer.setDirectPosition(startPosition + (targetPosition - startPosition) * shootTimer.getTime() / totalShootingTime);
                }
            }
            public boolean isFinished() {
                return shootTimer.getTime() > totalShootingTime && timeStarted;
            }

            public void end(boolean interrupted) {
                spindexer.setDirectPosition(targetPosition);
                stopItServo.setInactivePosition();
                spindexer.setDefault();
                shooter.transfer.stopMotor();
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


                // Drive to shoot
                new ParallelDeadlineGroup(
                        new LambdaCommand()
                                .setInitialize(() -> timer.restart())
                                .setExecute(() -> {
                                    calculateAlign(true);
                                    drive.pid(robotPose, new Pose(
                                            side.fromLeftX(shootAtPose2.getX()),
                                            shootAtPose2.getY(),
                                            targetHeading
                                    ), 1);
                                })
                                .setIsFinished(() -> ExtraFns.farZoneDist(robotPose) < 10),
                        new RunCommand(()-> setTransferPower())
                ),
                new InstantCommand(()-> pushUpServo.setUp()),
                new InstantCommand(() -> drive.stop()),
                new WaitCommand(500),
                new CommandBase() {
                    final BooleanSupplier shootSupplier = ExtraFns.firstSupplier(
                            () -> ((shooter.readyToShoot() && timer.getTime() > 1000)
                                    || timer.getTime() > 3000)
                    );
                    Timer shootTimer;
                    boolean timeStarted;
                    double startPosition = (SpindexerSpotNonCR.getPositionFromIndex(3, SpotType.INTAKE));
                    double targetPosition = 0;
                    public void initialize() {
                        timer.restart();
                        shootTimer = new Timer();
                        addRequirements(stopItServo, pushUpServo, shooter, turret);
                    }
                    boolean start;
                    public void execute() {
                        if(!start){
                            start = true;
                        }
                        calculateAlign(true);
                        setTransferPower();
                        if (shootSupplier.getAsBoolean() && !timeStarted) {
                            shootTimer = new Timer();
                            timeStarted = true;
                        }


                        //ready to shoot
                        if(timeStarted) {
                            spindexerPosition = startPosition + (targetPosition - startPosition) * Math.max(shootTimer.getTime(), totalShootingTime) / totalShootingTime;
                            spindexer.setDirectPosition(startPosition + (targetPosition - startPosition) * shootTimer.getTime() / totalShootingTime);
                        }
                    }
                    public boolean isFinished() {
                        return shootTimer.getTime() > totalShootingTime && timeStarted;
                    }

                    public void end(boolean interrupted) {
                        spindexer.setDirectPosition(targetPosition);
                        stopItServo.setInactivePosition();
                        spindexer.setDefault();
                        shooter.transfer.stopMotor();
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
                                    .setIsFinished(() -> timer.getTime() > 850 || follower.getVelocity().getMagnitude() < 5),
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
                                    .setIsFinished(() -> timer.getTime() > 300 && Math.abs(robotVel.getYComponent()) < 5),
                            // Drive forward again
                            new LambdaCommand()
                                    .setInitialize(() -> timer.restart())
                                    .setExecute(() -> drive.pid(
                                            robotPose,
                                            side.fromLeftPose(new Pose(-30, 0, headingFacingEdge)),
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
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign(true);
                            drive.pidNoHeading(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    zoneIntakeY + 8
                            ), 1);
                        })
                        .setIsFinished(() -> ExtraFns.farZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                        })
                        .setIsFinished(() -> timer.getTime() > 500)
        );

        Command balls7to9 = cornerIntakeShoot.apply(State.balls9);
        Command balls9to12 = cornerIntakeShoot.apply(State.balls12);
        Command balls12to15 = cornerIntakeShoot.apply(State.balls15);
        Command balls16to18 = cornerIntakeShoot.apply(State.balls18);

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
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                balls1To3,
                                balls4To6,
                                balls7to9,
                                balls9to12,
                                balls12to15,
                                driveToEnd,
                                stop
                        ),
                        new RunCommand(() -> setShooterPower(false))
                ),
                new InstantCommand(()-> stopFlywheels())
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

    public void stopFlywheels(){
        shooter.stopAll();
    }


    public double distToGoal() {
        return TwoWheelShooter2.getShootPose(side).distanceFrom(robotPose);
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

        telemetry.addLine("--------------------");

        telemetry.addData("State", state);
        telemetry.addData("Distance to goal", distToGoal());
        telemetry.addData("Spindexer position", spindexerPosition);

        telemetry.update();
    }
}