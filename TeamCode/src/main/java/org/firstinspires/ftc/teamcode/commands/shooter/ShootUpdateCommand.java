package org.firstinspires.ftc.teamcode.commands.shooter;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;

public class ShootUpdateCommand extends CommandBase {
    Spindexer spindexer;
    TwoWheelShooter shooter;
    SpindexerSpot[] seq;
    Follower follower;
    ShootSide shootSide;

    public boolean farthestMoved = false;
    TwoWheelShooter.ShootDist shootDist;

    boolean voltageUse = false;
    boolean useLUT = true;
    boolean rawPower;
    HardwareMap hardwareMap;

    boolean end;
    public ShootUpdateCommand(Spindexer spindexer, TwoWheelShooter shooter, Follower follower, ShootSide shootSide, boolean lutUse, boolean voltageUse, TwoWheelShooter.ShootDist shootDist, boolean rawPower, HardwareMap hardwareMap){
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.follower = follower;
        this.shootSide = shootSide;
        this.shootDist = shootDist;
        this.voltageUse = voltageUse;
        this.useLUT = lutUse;
        this.rawPower = rawPower;
        this.hardwareMap = hardwareMap;
        addRequirements(spindexer, shooter);
    }

    public ShootUpdateCommand(TwoWheelShooter shooter, HardwareMap hardwareMap, boolean end, Follower follower){
        this.hardwareMap = hardwareMap;
        this.shooter = shooter;
        this.rawPower = false;
        this.useLUT = true;
        this.follower = follower;
        this.end = end;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        double currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (useLUT) {
            shooter.setFlywheelStaticLUT(follower.getPose(), shootSide, voltageUse, currVolt);
        } else if(rawPower){
            shooter.setRawPower(1, 1);
        } else {
            shooter.setFlywheelStaticPresets(shootDist, voltageUse, currVolt);
        }

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        if(end) {
            shooter.stopFlywheels();
        }
    }
}
