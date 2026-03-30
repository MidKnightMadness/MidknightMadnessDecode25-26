package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter2;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@TeleOp
@Config
@Configurable
public class TurretSetAngleTest extends OpMode {
    public static ShootSide shootSide = ShootSide.LEFT;
    public static double radPerSec = 1.5;
    TelemetryManager telemetryM;
    WheelControl2 wheelControl;
    Follower follower;
    Turret turret;
    Pose robotPose;
    Timer timer;
    double goalHeading, turretTargetAngle;
    boolean manualMode = false;

    @Override
    public void init() {
        timer = new Timer();
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        wheelControl = new WheelControl2(hardwareMap);
        turret = new Turret(hardwareMap, true);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        robotPose = follower.getPose();
        updateTelemetry();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        robotPose = follower.getPose();
        wheelControl.driveRelative(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                1
        );

        if (gamepad1.leftBumperWasPressed()) {
            manualMode = !manualMode;
        }

        if (manualMode) {
            turretTargetAngle = Turret.angleRange.clip(
                    turretTargetAngle
                            + (gamepad1.right_trigger - gamepad1.left_trigger)
                            * radPerSec * timer.getDeltaTime(TimeUnit.SECONDS)
            );
        } else {
            goalHeading = TwoWheelShooter2
                    .getShootPose(shootSide)
                    .minus(robotPose)
                    .getAsVector()
                    .getTheta();

            turretTargetAngle = goalHeading - robotPose.getHeading();
        }

        turret.setAngleOptimized(
                Angle.fromRadians(turretTargetAngle)
        );

        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetryM.addData("Loop times (ms)", timer.getDeltaTime());
        telemetryM.addData("Pose X", robotPose.getX());
        telemetryM.addData("Pose Y", robotPose.getY());
        telemetryM.addData("Pose heading", robotPose.getHeading());
        telemetryM.addData("Turret position", turret.getPosition());
        telemetryM.addData("Turret angle (degrees)", turret.getAngleActual().toDegrees());
        telemetryM.addData("Turret target angle (degrees)", Math.toDegrees(turretTargetAngle));
        telemetryM.addData("Heading to goal", goalHeading);
        telemetryM.addData("Manual mode", manualMode);
        telemetryM.update(telemetry);
    }
}
