package org.firstinspires.ftc.teamcode.commands.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter2;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ExtraFns;
import org.firstinspires.ftc.teamcode.util.Timer;

public class ShootUpdateCommand extends CommandBase {
    SpindexerNonCR spindexer;
    TwoWheelShooter2 shooter;
    Turret turret;
    boolean sotmEnabled;
    Follower follower;
    ShootSide shootSide;
    VoltageSensor voltageSensor;
    double maxTime;
    public ShootUpdateCommand(SpindexerNonCR spindexer,
                              TwoWheelShooter2 shooter,
                              Turret turret,
                              Follower follower,
                              ShootSide shootSide,
                              VoltageSensor voltageSensor,
                              boolean sotmEnabled){
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.turret = turret;
        this.follower = follower;
        this.shootSide = shootSide;
        this.voltageSensor = voltageSensor;
        this.sotmEnabled = sotmEnabled;
        timer = new Timer();
        addRequirements(spindexer, shooter, turret);
    }


    @Override
    public void initialize(){
    }

    double targetHeading;
    Angle wrappedTurretValue;
    double turretHeadingError;
    Pose currentPose;
    double currVolt;
    Timer timer;
    @Override
    public void execute(){
        currentPose = follower.getPose();
        currVolt = voltageSensor.getVoltage();

        setTurret();
        setShooterPower();
    }


    public void setTurret(){
        if(sotmEnabled){
            double[] aimData;
            aimData = shooter.aimCalculator.targetPowersHeading(
                    follower.getPose(),
                    follower.getVelocity(),
                    TwoWheelShooter2.getShootPoseNew(currentPose, shootSide)
            );
            targetHeading = MathFunctions.normalizeAngle(aimData[2]);
        } else{
            targetHeading = TwoWheelShooter2.getShootHeading(follower.getPose(), shootSide);
        }
        wrappedTurretValue = Angle.fromRadians(ExtraFns.normAnglePlusMinusPI(targetHeading - currentPose.getHeading()));
        turret.setServos(turret.angleToServo(wrappedTurretValue));
        turretHeadingError = turret.getTurretHeadingError(wrappedTurretValue);
    }

    public void setShooterPower(){
        //set motors & transfer
        if(sotmEnabled) {
            shooter.setFlywheelNew(currentPose, follower.getVelocity(), shootSide, currVolt);
        } else{
            shooter.setFlywheelLUT(follower, shootSide, currVolt);
        }
        shooter.setTransferPower(TwoWheelShooter2.transferVelocity, currVolt);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
    }
}
