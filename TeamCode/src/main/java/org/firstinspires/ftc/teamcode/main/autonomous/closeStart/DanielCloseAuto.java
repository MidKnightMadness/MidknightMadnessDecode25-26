package org.firstinspires.ftc.teamcode.main.autonomous.closeStart;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LambdaCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
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

@Autonomous
@Configurable
public class DanielCloseAuto extends CommandOpMode {
    public static Pose shootPose = new Pose(58, 82);
    public static Pose startPose = new Pose(12, 130, Math.toRadians(135));
    public static double shootTimeout = 700;
    public static double rowXInner = 60;
    public static double rowXOuter = 25;
    public static double row1Y = 100;
    public static double row2Y = 70;
    public static double row3Y = 40;

    PIDController pidAutoAlign = new PIDController(1.0, 0, 0.1);
    Follower follower;
    ShootSide shootSide = ShootSide.LEFT;
    TelemetryManager telemetryM;
    WheelControl2 drive;
    Timer timer = new Timer(TimeUnit.MILLISECONDS);
    TwoWheelShooter shooter;
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
        end
    }
    State state;

    @Override
    public void initialize() {
        Robot.config = AllConfigs.oldBot;
        state = State.init;
        follower = ConstantsOldBot.createPinpointFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        drive = new WheelControl2(hardwareMap);

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
            main.schedule();
            started = true;
        }
        updateTelemetry();
    }

    public void calculateAlign() {
        aimData = shooter.aimCalculator.targetPowersHeading(
                follower.getPose(),
                follower.getVelocity(),
                TwoWheelShooter.getShootPose(shootSide)
        );
        targetHeading = aimData[2];
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
            final BooleanSupplier shootSupplier = ExtraFns.risingEdge(
                    () -> distToGoal() > 50 || timer.getTime() > 700
            );
            final Timing.Timer shootTimer = new Timing.Timer(400, TimeUnit.MILLISECONDS);
            public void initialize() {
                timer.restart();
                state = State.balls3;
                addRequirements(shooter);
            }
            public void execute() {
                calculateAlign();
                drive.driveRelative(-1, 0, pidAutoAlign.calculate(robotPose.getHeading() - targetHeading), 1);
                if (shootSupplier.getAsBoolean()) {
                    shootTimer.start();
                    shooter.setCustomPower(aimData[0], aimData[1], currVolt);
                }
            }
            public boolean isFinished() {
                return shootTimer.done() || distToGoal() > 100;
            }
        };

        Command balls4To6 = new SequentialCommandGroup(
                new InstantCommand(() -> state = State.balls6),
                // Full send to almost there
                new LambdaCommand()
                    .setInitialize(() -> timer.restart())
                    .setExecute(() -> drive.pid(
                            robotPose, new Pose(rowXInner + 15, row2Y), 1)
                    )
                    .setIsFinished(() -> robotPose.getY() < row2Y + 10),
                // Adjust position and power
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXInner, row2Y), 0.5)
                        )
                        .setIsFinished(() -> robotPose.getX() < rowXInner),
                // Drive straight forward and intake
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXOuter, row2Y), 0.5)
                        )
                        .setIsFinished(() -> robotPose.getX() < rowXOuter),
                // Drive straight back
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> drive.pid(
                                robotPose, new Pose(rowXOuter, row2Y), 1)
                        )
                        .setIsFinished(() -> robotPose.getX() > rowXOuter - 5),
                // Drive to shoot
                new LambdaCommand()
                        .setInitialize(() -> timer.restart())
                        .setExecute(() -> {
                            calculateAlign();
                            drive.pid(robotPose, new Pose(shootPose.getX(), shootPose.getY(), targetHeading), 1);
                        })
                        .setIsFinished(() -> ExtraFns.closeZoneDist(robotPose) < 8)
        );

        main = new SequentialCommandGroup(
                balls1To3,
                balls4To6,
                new InstantCommand(() -> {
                    drive.stop();
                    state = State.end;
                })
        );
    }

    public double distToGoal() {
        return TwoWheelShooter.getShootPose(shootSide).distanceFrom(robotPose);
    }

    public void updateTelemetry() {
        telemetryM.addData("Robot pose X", robotPose.getX());
        telemetryM.addData("Robot pose Y", robotPose.getY());
        telemetryM.addData("Robot pose heading", Math.toDegrees(robotPose.getHeading()));
        telemetryM.addData("Robot velocity X", robotVel.getXComponent());
        telemetryM.addData("Robot velocity Y", robotVel.getYComponent());

        telemetryM.addLine("--------------------");

        telemetryM.addData("State", state);
        telemetryM.addData("Distance to end", distToGoal());
        telemetryM.update(telemetry);
    }
}
