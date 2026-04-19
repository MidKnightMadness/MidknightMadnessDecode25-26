package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.right;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@Autonomous
public class DanielCloseAutoRight2 extends CommandOpMode {
    public static Pose shootAtPose1 = new Pose(48, 92).mirror();
    public static Pose shootAtPose2 = new Pose(55, 83).mirror();
    public static Pose gateIntakePose = new Pose(3, 60, Math.toRadians(150)).mirror();
    public static Pose startPose = new Pose(22, 120, Math.toRadians(142)).mirror();
    public static double shootTimeout = 700;
    public static double rowXInner = 141.5 - 40;
    public static double rowXOuter = 141.5 - 25;
    public static double row1Y = 38;
    public static double row2Y = 62;
    public static double row3Y = 85;
    public static double headingFacingEdge = 0;
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
        jank1,
        jank2,
        jank3,
        jank4,
        jank5,
        jank6,
        balls9,
        balls12,
        balls15,
        balls18,
        balls21,
        end
    }
    State state;

    public ShootSide getShootSide() {
        return ShootSide.RIGHT;
    }

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
                                shootAtPose1.getX(),
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
                        .setInitialize(() -> {timer.restart(); state = State.jank1;})
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        rowXInner,
                                        row2Y,
                                        headingFacingEdge
                                ), 1)
                        )
                        .setIsFinished(() -> robotPose.getY() < row2Y + 10),
                // Adjust position and power
                new LambdaCommand()
                        .setInitialize(() -> {timer.restart(); state = State.jank2;})
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(
                                        rowXInner,
                                        row2Y,
                                        headingFacingEdge
                                ), 0.5)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXInner - 1),
                // Drive straight forward and intake
                new LambdaCommand()
                        .setInitialize(() -> {timer.restart(); state = State.jank3;})
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXOuter, row2Y, headingFacingEdge), 0.5)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXOuter),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> {timer.restart(); state = State.jank4;})
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner, row2Y, headingFacingEdge), 1)
                        )
                        .setIsFinished(() -> robotPose.getX() < rowXInner + 14),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> {timer.restart(); state = State.jank5;})
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(
                                    shootAtPose2.getX(),
                                    shootAtPose2.getY(),
                                    targetHeading
                            ), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 7),
                // TODO: actually shoot
                new LambdaCommand()
                        .setInitialize(() -> {
                            timer.restart();
                            drive.stop();
                            state = State.jank6;
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
