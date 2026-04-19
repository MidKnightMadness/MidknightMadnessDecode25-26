package org.firstinspires.ftc.teamcode.main.autonomous.farStart.base;

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
import java.util.function.Function;

@Configurable
public abstract class DanielFarAutoClean extends CommandOpMode {
//    public static Pose shootAtPose1 = new Pose(48, 92);
    public static Pose shootAtPose2 = new Pose(60, 10);
    public static Pose gateIntakePose = new Pose(3, 60, Math.toRadians(150));
    public static Pose startPose = new Pose(55, 8, Math.toRadians(90));
    public static Pose endPose = new Pose(30, 34);
    public static double shootTimeout = 700;
    public static double rowXInner = 35;
    public static double rowXOuter = 20;
    public static double row1Y = 45;
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
    double zoneIntakeY = 27;

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
        Command balls1To3 = new InstantCommand(() -> state = State.balls3); // fix later

        Command balls4To6 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls6),
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
                // Drive straight forward and intake
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(side.fromLeftX(rowXOuter), row1Y, side.fromLeftHeading(headingFacingEdge)), 0.5)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXOuter),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    shootAtPose2.getY(),
                                    targetHeading
                            ), 1);
                        })
                        .setIsFinished(() -> ExtraFns.farZoneDist(robotPose) < 10),
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
                                robotPose, side.fromLeftPose(new Pose(
                                        rowXOuter,
                                        zoneIntakeY,
                                        headingFacingEdge
                                )), 1)
                        )
                        .setIsFinished(() -> side.toLeftX(robotPose.getX()) < rowXOuter),
                // Drive to corner
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose,
                                side.fromLeftPose(new Pose(0, zoneIntakeY, headingFacingEdge)),
                                0.5
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
                        .setIsFinished(() -> robotPose.getX() > cornerX + 5),
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
                                side.fromLeftPose(new Pose(-100, 0, headingFacingEdge)),
                                0.5
                        ))
                        .setIsFinished(() -> timer.getTime() > 300 && follower.getVelocity().getMagnitude() < 5),
//                // Wait for gate intake
//                new InstantCommand(() -> drive.stop()),
//                new WaitCommand(700),
//                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(
                                    side.fromLeftX(shootAtPose2.getX()),
                                    zoneIntakeY - 8,
                                    targetHeading
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

        Command balls7to9 = gateIntakeShoot.apply(State.balls9);
        Command balls9to12 = gateIntakeShoot.apply(State.balls12);
        Command balls12to15 = gateIntakeShoot.apply(State.balls15);

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
                driveToEnd,
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
