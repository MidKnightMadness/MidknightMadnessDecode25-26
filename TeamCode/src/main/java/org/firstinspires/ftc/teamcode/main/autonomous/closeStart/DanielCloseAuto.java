package org.firstinspires.ftc.teamcode.main.autonomous.closeStart;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LambdaCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.commands.Robot;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.AllConfigs;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@Autonomous
@Configurable
public class DanielCloseAuto extends CommandOpMode {
    public static Pose startPose = new Pose(12, 130, Math.toRadians(135));
    public static double shootTimeout = 700;
    public static double rowXInner = 60;
    public static double rowXOuter = 25;
    public static double row1Y = 100;
    public static double row2Y = 70;
    public static double row3Y = 40;

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

    public void updateData() {
        follower.update();
        robotPose = follower.getPose();
        robotVel = follower.getVelocity();
        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void initCommands() {
        Supplier<Command> balls1To3 = () -> {
            Timing.Timer shootTimer = new Timing.Timer(400, TimeUnit.MILLISECONDS);
            return new LambdaCommand()
                    .setInitialize(() -> {
                        timer.restart();
                        state = State.balls3;
                    })
                    .setExecute(() -> {
                        drive.driveRelative(-1, 0, 0, 1);
                        if (distToGoal() > 50 || timer.getTime() > 700) {
                            shootTimer.start();
//                        shooter.setFlywheelNew(robotPose, robotVel, shootSide, currVolt);
                        }
                    })
                    .setIsFinished(() -> (shootTimer.done() || distToGoal() > 100))
                    .addRequirements(shooter);
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
                // Drive straight forward
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
                        .setIsFinished(() -> robotPose.getX() > rowXOuter - 5)
        );

        main = new SequentialCommandGroup(
                balls1To3.get(),
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
