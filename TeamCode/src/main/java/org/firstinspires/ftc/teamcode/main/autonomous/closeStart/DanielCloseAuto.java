package org.firstinspires.ftc.teamcode.main.autonomous.closeStart;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.LambdaCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.commands.Robot;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.AllConfigs;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous
@Configurable
public class DanielCloseAuto extends CommandOpMode {
    public static Pose startPose = new Pose(12, 130, Math.toRadians(135));

    Follower follower;
    ShootSide shootSide = ShootSide.LEFT;
    TelemetryManager telemetryM;
    WheelControl2 wheelControl;
    Timer timer = new Timer(TimeUnit.MILLISECONDS);
    Timing.Timer firstTimeout;
    TwoWheelShooter shooter;
    SequentialCommandGroup main;
    Pose robotPose;
    Vector robotVel;
    double currVolt;
    boolean started = false;

    Command backUp;
    enum State {
        init,
        firstBack,
        end
    }
    State state;

    @Override
    public void initialize() {
        Robot.config = AllConfigs.oldBot;
        state = State.init;
        follower = ConstantsOldBot.createPinpointFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        wheelControl = new WheelControl2(hardwareMap);

        firstTimeout = new Timing.Timer(1);
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
        backUp = new LambdaCommand()
                .setInitialize(() -> {
                    timer.restart();
                    state = State.firstBack;
                })
                .setExecute(() -> {
                    wheelControl.drive_relative(-1, 0, 0, 1);
                    if (distToGoal() > 50 || timer.getTime() > 700) {
                        firstTimeout.start();
//                        shooter.setFlywheelNew(robotPose, robotVel, shootSide, currVolt);
                    }
                })
                .setIsFinished(() -> (firstTimeout.done() || distToGoal() > 100))
                .addRequirements(shooter);

        main = new SequentialCommandGroup(
                backUp,
                new InstantCommand(() -> {
                    wheelControl.stop();
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
