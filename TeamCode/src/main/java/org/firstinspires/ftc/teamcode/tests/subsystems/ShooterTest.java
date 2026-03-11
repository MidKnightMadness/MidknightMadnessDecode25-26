package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;


@TeleOp(name = "Shooter Test")
@Disabled
@Configurable
public class ShooterTest extends OpMode {
    TwoWheelShooter shooter;
    public static TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;
    public static boolean voltageCompensation = false;
    public static boolean useLUT = false;
    CRServo spindexerServo;
    CRServo spindexerServo2;
    Telemetry dashboardTelemetry;
    VoltageSensor voltageSensor;


    double maxTurnerSpeed = 1;
    public static double currturnerSpeed = 0.3;//0.3 before
    double midTurnerSpeed = 0.5;
    boolean shootOn;
    boolean readyToShoot = false;
    public static CRServoEx2.RunMode spindexerRunMode = CRServoEx2.RunMode.OptimizedPositionalControl;

    public static boolean setCustomPower = false;
    public static double customTopTargetVel = 1700;
    public static double customBotTargetVel = 1400;
    Timer gameTimer;


    public static double change = 1;
    boolean triggerBallShot = false;
    int recentTriggeredSpot = -1;
    int triggeredSpot = -1;
    boolean velAgressiveComp = false;
    Spindexer spindexer;
    boolean useColorSensor = false;
    boolean useDistanceSensor = false;
    public static double customTopPower = 0;
    public static double customBotPower = 0;


    @Override
    public void init() {
        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
        spindexer = new Spindexer(hardwareMap, useDistanceSensor, new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}).initAngle();
        spindexer.setMode(spindexerRunMode);
        spindexerServo = spindexer.getTurner().getServo();
        gameTimer = new Timer();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

    }

    boolean recoveryOn = false;
    double currVolt;
    @Override
    public void loop() {
        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();

        spindexerServo.setPower(gamepad2.left_stick_y * currturnerSpeed * change);
//        spindexerServo2.setPower(gamepad2.left_stick_y * currturnerSpeed * change);

        updateTelem();
        handleShooterInput();


        if(shooter.low.getVelocity() > 200  || shooter.low.motor.getPower() > 0.1){
            shootOn = true;
        } else{
            shootOn = false;
        }
        spindexer.updateShootOn(shootOn);


        if(!shootOn){
            return;
        }


        if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT0, SpotType.OUTTAKE)){
            triggeredSpot = 0;
            recentTriggeredSpot = triggeredSpot;
            velAgressiveComp = true;
        } else if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT1, SpotType.OUTTAKE)){
            triggeredSpot = 1;
            recentTriggeredSpot = triggeredSpot;
            velAgressiveComp = true;
        } else if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT2, SpotType.OUTTAKE)){
            triggeredSpot = 2;
            recentTriggeredSpot = triggeredSpot;
            velAgressiveComp = true;
        } else{
            triggeredSpot = -1;
            triggerBallShot = false;
        }
        if(!triggerBallShot && triggeredSpot != -1){
            shooter.triggerBallShot();
            spindexer.removeBall(triggeredSpot);
            triggerBallShot = true;
        }





    }

    boolean prevLeftTrigger = false;
    private void handleShooterInput(){
        if (gamepad2.left_trigger > 0.5 && !prevLeftTrigger) {
            prevLeftTrigger = true;
            shooterRunMode = shooterRunMode == TwoWheelShooter.RunMode.RawPower ? TwoWheelShooter.RunMode.VelocityControl : TwoWheelShooter.RunMode.RawPower;
            shooter.setRunMode(shooterRunMode);
        } else if(gamepad2.left_trigger < 0.5){
            prevLeftTrigger = false;
        }


        if (gamepad2.left_bumper) {
            setCurrentShootDist(TwoWheelShooter.ShootDist.Close);
        } else if (gamepad2.right_bumper) {
            setCurrentShootDist(TwoWheelShooter.ShootDist.Far);
        }

        if(gamepad2.optionsWasPressed()){
            useLUT = ! useLUT;
        }
        if(gamepad2.shareWasPressed()){
            voltageCompensation = ! voltageCompensation;
            if(voltageCompensation) gamepad2.rumbleBlips(2);
        }

        if (gamepad2.right_trigger > 0.5) {
            shooter.stopFlywheels();
        }
    }

    TwoWheelShooter.ShootDist currentShootDist;
    private void setCurrentShootDist(TwoWheelShooter.ShootDist shootDist) {
        currentShootDist = shootDist;
        if (!setCustomPower) {
//            if(useLUT){
//                shooter.setFlywheelLUT(follower, shootSide, voltageCompensation);
//            }
            shooter.resetDefaultGains();
            shooter.setFlywheelStaticPresets(shootDist, voltageCompensation, currVolt);
        } else{
            if(shooterRunMode == TwoWheelShooter.RunMode.VelocityControl) {
                shooter.setCustomPower(customBotTargetVel, customTopTargetVel, currVolt);
            } else{
                shooter.setCustomPower(customBotPower, customTopPower, currVolt);
            }
        }
    }

    private void updateTelem() {
        telemetry.addData("update rate", (double) 1000.0 / gameTimer.getDeltaTime());


        if (shooterRunMode == TwoWheelShooter.RunMode.RawPower) {
            telemetry.addData("Top motor voltage", customTopPower * currVolt);
            telemetry.addData("Bottom motor voltage", customTopPower * currVolt);
        }
        telemetry.addData("Voltage Use", voltageCompensation);
        telemetry.addData("Use LUT", useLUT);
        telemetry.addData("Shoot Mode", shooterRunMode);
        telemetry.addData("Current Voltage", shooter.getCurrVoltage());
        telemetry.addData("Ratio Voltage ", shooter.getTargetVoltage() / shooter.getCurrVoltage());

        telemetry.addData("Shooter Top Factor", shooter.getCurrTopFactor());
        telemetry.addData("Shooter Bot Factor", shooter.getCurrBotFactor());
        telemetry.addData("Multiplier Top", shooter.getCurrTopFactor());
        telemetry.addData("Multiplier Bot", shooter.getCurrBotFactor()
        );
        telemetry.addData("Trigger Ball Shot", triggerBallShot);
        telemetry.addData("Recently Triggered Shot", recentTriggeredSpot);
        telemetry.addData("Ready to Shoot", shooter.readyToShoot());
        telemetry.addData("Actual Recovery Time", shooter.getRecoveryTime());


        telemetry.addLine("--------------------------------");
        telemetry.addData("Shooter Mode", shooterRunMode);
        telemetry.addData("Shooter Top Power", shooter.high.get());
        telemetry.addData("Shooter Bot Power", shooter.low.get());
        telemetry.addData("Shooter Top Vel", shooter.high.getVelocity());
        telemetry.addData("Shooter Bot Vel", shooter.low.getVelocity());
        telemetry.addData("Corr Shooter Top", shooter.high.getCorrectedVelocity());
        telemetry.addData("Corr Shooter Bot", shooter.low.getCorrectedVelocity());


        dashboardTelemetry.addData("Shoter Top Vel", shooter.high.getVelocity());
        dashboardTelemetry.addData("Shoter Bot Vel", shooter.low.getVelocity());
        dashboardTelemetry.addData("Shooter Corr Top Vel", shooter.high.getCorrectedVelocity());
        dashboardTelemetry.addData("Shooter Corr Bot Vel", shooter.low.getCorrectedVelocity());

        telemetry.addLine("--------------------------------");
        telemetry.addData("Spindexer Mode", spindexerRunMode);
        telemetry.addData("Spindexer Angle", spindexer.getCurrentAngle());
        telemetry.addData("Spindexer Angle Error", spindexer.getTurner().error);

        telemetry.update();
        dashboardTelemetry.update();

    }
}
