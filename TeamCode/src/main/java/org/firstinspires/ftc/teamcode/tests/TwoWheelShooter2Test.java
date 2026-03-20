package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter2;
import org.firstinspires.ftc.teamcode.util.AngleNonCR;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;

@TeleOp(name = "Two Wheel Shooter 2 Test")
@Config
@Configurable
public class TwoWheelShooter2Test extends OpMode {
    TwoWheelShooter2 shooter;
//    Turret turret;
    ServoImplEx turner;
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;
    TwoWheelShooter2.RunMode shooterRunMode = TwoWheelShooter2.RunMode.VelocityControl;
    TwoWheelShooter2.RunMode transferRunMode = TwoWheelShooter2.RunMode.VelocityControl;
    public static double lowVelocity = 1150;
    public static double highVelocity = 1600;
    public static double lowPower = 1;
    public static double highPower = 1;

    public static double transferPower = 0.7;
    public static double transferVelocity = 1500;
    Telemetry dashboardTelemetry;
    public static boolean useAgressiveRecovery = true;

    public static double targetTurretAngle = 90;
    public static boolean useBulkMode = true;
    Timer timer;
    @Override
    public void init() {
        shooter = new TwoWheelShooter2(hardwareMap, shooterRunMode, transferRunMode);
        shooter.setAggressiveRecovery(useAgressiveRecovery);
        leftServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoLeft);
        rightServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoRight);
//        turret = new Turret(hardwareMap, true);
        timer = new Timer();

        turner = hardwareMap.get(ServoImplEx.class, ConfigNames.turner);
        turner.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turner.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        if(useBulkMode) {
//            PhotonCore.disable();
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
            );
        }
        else {
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
            );
        }

    }

    @Override
    public void loop() {
        double currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if(shooterRunMode == TwoWheelShooter2.RunMode.VelocityControl){
            shooter.setCustomPower(lowVelocity, highVelocity, currVolt);
        } else{
            shooter.low.set(lowPower);
            shooter.high.set(highPower);
        }

        if(transferRunMode == TwoWheelShooter2.RunMode.RawPower){
            shooter.transfer.set(transferPower, currVolt);
        } else{
            shooter.setTransferPower(transferVelocity, currVolt);
        }
        if(useAgressiveRecovery){
            shooter.updateRecoveryState();
        }

        leftServo.setPosition((1 + gamepad1.left_stick_x) / 2);
        rightServo.setPosition((1 + gamepad1.left_stick_x) / 2);

        if(gamepad1.leftBumperWasPressed()){
            turner.setPosition(0);
        }
        if(gamepad1.rightBumperWasPressed()){
            turner.setPosition(1);
        }

//        turret.setServos(turret.angleToServo(AngleNonCR.fromDegrees(targetTurretAngle)));



        telemetry.addData("Update Rate", 1.0 / timer.getDeltaTime());
        telemetry.addLine("-----------------------------------------------");
        telemetry.addData("Shooter RunMode", shooterRunMode);
        telemetry.addData("Low Velocity", shooter.low.getVelocity());
        telemetry.addData("High Velocity", shooter.high.getVelocity());
        telemetry.addData("Transfer Velocity", shooter.transfer.getVelocity());
        telemetry.addData("Low Error", (shooter.bottomError));
        telemetry.addData("High Error", (shooter.topError));
        telemetry.addData("Transfer Error", (shooter.transferError));
        dashboardTelemetry.addData("Low Error", shooter.bottomError);
        dashboardTelemetry.addData("High Error", shooter.topError);
        dashboardTelemetry.addData("Transfer Error", (shooter.transferError));

        telemetry.addLine("-----------------------------------------------");
//        telemetry.addData("Turret Servo Left Position", turret.getServoLeftPosition());
//        telemetry.addData("Turret Servo Right Position", turret.getServoRightPosition());
//        telemetry.addData("Encoder Angle", turret.getEncoder().getAngleUnormalizedEncoder());
//        telemetry.addData("Current Encoder Servo Position", turret.angleToServo(AngleNonCR.fromDegrees(turret.getEncoder().getAngleUnormalizedEncoder())));

        dashboardTelemetry.update();
        telemetry.update();
    }
}
