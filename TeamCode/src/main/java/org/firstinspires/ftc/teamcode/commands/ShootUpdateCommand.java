package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;

public class ShootUpdateCommand extends CommandBase {
    Spindexer spindexer;
    TwoWheelShooter shooter;
    SpindexerSpot[] seq;
    Follower follower;
    ShootSide shootSide;
    public int currBallIndex = 0;
    public static double angleTolerance = 20;
    boolean mapDistToShoot;
    Timer timer;

    public static double goToGreenSpotWait = 1000;
    public static double flywheelSpinupWait = 2500;
    public static double betweenShotsWait = 500;
    public static double betweenShotsWaitArr[] = new double[]{2000, 1000, 1000};
    boolean flywheelSpinupStarted = false;

    //    public boolean spinStarted = false;
    boolean spinEnded = false;
    boolean removedBall = false;
    double spinStartTime = 0;
    boolean lastBallRemoved = false;

    public boolean farthestMoved = false;
    boolean powerFlywheel;
    TwoWheelShooter.ShootDist shootDist;
    public static double maxTimeShoot = 8000;

    boolean voltageUse = false;
    boolean useLUT = true;
    boolean rawPower;
    public int getCurrBallIndex(){
        return currBallIndex;
    }

    public ShootUpdateCommand(Spindexer spindexer, TwoWheelShooter shooter, Follower follower, ShootSide shootSide, boolean lutUse, boolean voltageUse, TwoWheelShooter.ShootDist shootDist, boolean rawPower){
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.follower = follower;
        this.shootSide = shootSide;
        this.shootDist = shootDist;
        this.voltageUse = voltageUse;
        this.useLUT = lutUse;
        this.rawPower = rawPower;
        addRequirements(spindexer, shooter);
        timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public void execute(){
        if(rawPower){
            shooter.setRawPower(1, 1);
        } else if (useLUT) {
            shooter.setFlywheelStaticLUT(follower.getPose(), shootSide, voltageUse);
        } else {
            shooter.setFlywheelStaticPresets(shootDist, voltageUse);
        }


    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopFlywheels();
//        spindexer.goToSpot(SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        spindexer.getTurner().getServo().setPower(0);
        spindexer.getTurner2().getServo().setPower(0);
    }
}
