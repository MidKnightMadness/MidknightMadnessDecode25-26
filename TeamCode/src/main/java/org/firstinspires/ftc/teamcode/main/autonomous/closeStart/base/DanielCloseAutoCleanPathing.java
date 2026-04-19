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
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.AllConfigs;
import org.firstinspires.ftc.teamcode.util.ExtraFns;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

@Configurable
public abstract class DanielCloseAutoCleanPathing extends CommandOpMode {
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
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        drive = new WheelControl2(hardwareMap)
                .setPidControllers(driveController, headingController);

        follower.setStartingPose(side.fromLeftPose(startPose));
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
                drive.pidNoHeading(
                        robotPose,
                        new Pose(
                                side.fromLeftX(shootAtPose1.getX()),
                                shootAtPose1.getY()
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
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXOuter), row2Y, side.fromLeftHeading(headingFacingEdge)), 0.5)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXOuter),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXInner), row2Y, side.fromLeftHeading(headingFacingEdge)), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > rowXInner - 14),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pidNoHeading(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    shootAtPose2.getY()
                            ), 1);
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
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(robotPose, side.fromLeftPose(gateIntakePose)))
                        .setIsFinished(() -> timer.getTime() > 1000 || follower.getVelocity().getMagnitude() < 5),
                // Wait for gate intake
                new InstantCommand(() -> drive.stop()),
                new WaitCommand(700),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pidNoHeading(
                                robotPose, new Pose(side.fromLeftX(rowXInner), row2Y), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > rowXInner - 9),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pidNoHeading(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    shootAtPose2.getY()
                            ), 1);
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
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXInner), row3Y, side.fromLeftHeading(headingFacingEdge)), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) > rowXInner - 14),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pidNoHeading(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    shootAtPose2.getY()
                            ), 1);
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
        telemetryM.addData("Auto time elapsed", autoElapsed.getTime());
        telemetryM.addData("Timer", timer.getTime());
        telemetryM.addData("Robot pose X", robotPose.getX());
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