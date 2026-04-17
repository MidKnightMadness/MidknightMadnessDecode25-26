package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LambdaCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.commands.Robot;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.newpid.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.AllConfigs;
import org.firstinspires.ftc.teamcode.util.ExtraFns;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

@Configurable
public abstract class DanielCloseAuto extends CommandOpMode {
    public static Pose shootAtPose1 = new Pose(48, 92);
    public static Pose shootAtPose2 = new Pose(55, 83);
    public static Pose gateIntakePose = new Pose(3, 60, Math.toRadians(150));
    public static Pose startPose = new Pose(22, 120, Math.toRadians(142));
    public static double headingToEdge = -Math.PI;
    public static double shootTimeout = 700;
    public static double rowXInner = 40;
    public static double rowXOuter = 25;
    public static double row1Y = 38;
    public static double row2Y = 62;
    public static double row3Y = 85;
    PIDController driveController = new PIDController(0.05, 0, 0.001);
    PIDController headingController = new PIDController(1.0, 0, 0.01);

    PIDController pidAutoAlign = new PIDController(1.0, 0, 0.1);
    Follower follower;
    ShootSide side;
    TelemetryManager telemetryM;
    WheelControl2 drive;
    Timer timer = new Timer(TimeUnit.MILLISECONDS);
    Timer autoElapsed = new Timer(TimeUnit.SECONDS);
//    TwoWheelShooter shooter;
    SequentialCommandGroup main;
    Pose robotPose;
    Vector robotVel;
    double currVolt;
    boolean started = false;
    double targetHeading;
    double headingError;
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
    State state;

    public abstract ShootSide getShootSide();

    @Override
    public void initialize() {
        Robot.config = AllConfigs.oldBot;

        side = getShootSide();

        state = State.init;
        follower = ConstantsOldBot.createPinpointFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        drive = new WheelControl2(hardwareMap)
                .setPidControllers(driveController, headingController);

        follower.setStartingPose(startPose);
//        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);
        initCommands();
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

    public void calculateAlign() {
//        aimData = shooter.aimCalculator.targetPowersHeading(
//                follower.getPose(),
//                follower.getVelocity(),
//                TwoWheelShooter.getShootPose(shootSide)
//        );
//        targetHeading = aimData[2];
//        headingError = MathFunctions.normalizeAngleSigned(
//                follower.getPose().getHeading() - targetHeading
//        );
        targetHeading = TwoWheelShooter.getShootPose(side).minus(robotPose).getAsVector().getTheta();
        headingError = MathFunctions.normalizeAngleSigned(
                follower.getPose().getHeading() - targetHeading
        );
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
                    () -> distToGoal() > 30 || timer.getTime() > 350
            );
            final Timing.Timer shootTimer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
            public void initialize() {
                timer.restart();
                state = State.balls3;
//                addRequirements(shooter);
            }
            public void execute() {
                calculateAlign();
                drive.pid(
                        robotPose,
                        new Pose(
                            side.fromLeftX(shootAtPose1.getX()),
                            shootAtPose1.getY(),
                            targetHeading
                        )
                );
                if (shootSupplier.getAsBoolean()) {
                    shootTimer.start();
//                    shooter.setCustomPower(aimData[0], aimData[1], currVolt);
                }
            }
            public boolean isFinished() {
                return shootTimer.done();
            }
        };

        Command balls4To6 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls6),
                // Full send to almost there
                new LambdaCommand()
                    .setInitialize(() -> timer.restart())
                    .setExecute(() -> drive.pid(
                            robotPose, new Pose(
                                    side.fromLeftX(rowXInner), row2Y, headingToEdge
                            ), 1)
                    )
                    .setIsFinished(() -> robotPose.getY() < row2Y + 10),
                // Adjust position and power
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        side.fromLeftX(rowXInner), row2Y, headingToEdge
                                ), 0.5)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXInner + 1),
                // Drive straight forward and intake
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXOuter), row2Y, headingToEdge), 0.5)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXOuter),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXInner), row2Y, headingToEdge), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > rowXInner - 16),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(shootAtPose2.getX(), shootAtPose2.getY(), targetHeading), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                        })
                        .setIsFinished(() -> timer.getTime() > 500)
        );

        Command balls7To9 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls9),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner, gateIntakePose.getY(), gateIntakePose.getHeading()), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() < gateIntakePose.getY() + 10),
                // Drive to gate
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(robotPose, gateIntakePose))
                        .setIsFinished(() -> timer.getTime() > 850 || follower.getVelocity().getMagnitude() < 5),
                // Wait for gate intake
                new InstantCommand(() -> drive.stop()),
                new WaitCommand(700),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pidNoHeading(
                                robotPose, new Pose(rowXInner, row2Y), 1)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXInner - 9),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(shootAtPose2.getX(), shootAtPose2.getY(), targetHeading), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                        })
                        .setIsFinished(() -> timer.getTime() > 500)
        );

        Command balls10To12 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls12),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner, gateIntakePose.getY(), gateIntakePose.getHeading()), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() < gateIntakePose.getY() + 10),
                // Drive to gate
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(robotPose, gateIntakePose))
                        .setIsFinished(() -> timer.getTime() > 850 || follower.getVelocity().getMagnitude() < 5),
                // Wait for gate intake
                new InstantCommand(() -> drive.stop()),
                new WaitCommand(700),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pidNoHeading(
                                robotPose, new Pose(rowXInner, row2Y), 1)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXInner - 9),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(shootAtPose2.getX(), shootAtPose2.getY(), targetHeading), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                        })
                        .setIsFinished(() -> timer.getTime() > 500)
        );

        Command balls13To15 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls15),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner, gateIntakePose.getY(), gateIntakePose.getHeading()), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() < gateIntakePose.getY() + 10),
                // Drive to gate
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(robotPose, gateIntakePose))
                        .setIsFinished(() -> timer.getTime() > 850 || follower.getVelocity().getMagnitude() < 5),
                // Wait for gate intake
                new InstantCommand(() -> drive.stop()),
                new WaitCommand(700),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pidNoHeading(
                                robotPose, new Pose(rowXInner, row2Y), 1)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXInner - 9),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(shootAtPose2.getX(), shootAtPose2.getY(), targetHeading), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                        })
                        .setIsFinished(() -> timer.getTime() > 500)
        );

        Command balls16To18 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls18),
                // Full send to almost there
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner, gateIntakePose.getY(), gateIntakePose.getHeading()), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() < gateIntakePose.getY() + 10),
                // Drive to gate
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(robotPose, gateIntakePose))
                        .setIsFinished(() -> timer.getTime() > 850 || follower.getVelocity().getMagnitude() < 5),
                // Wait for gate intake
                new InstantCommand(() -> drive.stop()),
                new WaitCommand(700),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pidNoHeading(
                                robotPose, new Pose(rowXInner, row2Y), 1)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXInner - 9),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(shootAtPose2.getX(), shootAtPose2.getY(), targetHeading), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                        })
                        .setIsFinished(() -> timer.getTime() > 500)
        );

        Command balls19To21 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls21),
                // Drive just before intake
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner + 2, row3Y, headingToEdge), 0.5)
                        )
                        .setIsFinished(() -> robotPose.getY() < row3Y + 1),
                // Drive straight forward and intake
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXOuter, row3Y, headingToEdge), 0.5)
                        )
                        .setIsFinished(() -> robotPose.getX() < rowXOuter),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner, row3Y, headingToEdge), 1)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXInner - 16),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(shootAtPose2.getX(), shootAtPose2.getY(), targetHeading), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                        })
                        .setIsFinished(() -> timer.getTime() > 500)
        );

        main = new SequentialCommandGroup(
                balls1To3,
                balls4To6,
                balls7To9,
                balls10To12,
                balls13To15,
                balls16To18,
                balls19To21,
                new InstantCommand(() -> {
                    drive.stop();
                    state = State.end;
                })
        );
    }

    public double distToGoal() {
        return TwoWheelShooter.getShootPose(side).distanceFrom(robotPose);
    }

    public void updateTelemetry() {
        telemetryM.addData("start pose", startPose);
        telemetryM.addData("Auto time elapsed", autoElapsed.getTime());
        telemetryM.addData("Timer", timer.getTime());
        telemetryM.addData("Robot pose X", robotPose.getX());
        telemetryM.addData("Robot pose Y", robotPose.getY());
        telemetryM.addData("Robot pose heading", Math.toDegrees(robotPose.getHeading()));
        telemetryM.addData("Robot velocity X", robotVel.getXComponent());
        telemetryM.addData("Robot velocity Y", robotVel.getYComponent());

        telemetryM.addLine("--------------------");

        telemetryM.addData("State", state);
        telemetryM.addData("Distance to goal", distToGoal());
        telemetryM.update(telemetry);
    }
}
