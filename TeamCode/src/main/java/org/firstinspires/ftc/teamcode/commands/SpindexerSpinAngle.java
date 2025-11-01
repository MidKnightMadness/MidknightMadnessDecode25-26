package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

public class SpindexerSpinAngle extends CommandBase {
    public static double stopThreshold = 20;

    private double targetAngle;
    private final double power;
    private final Angle angle;
    private final Spindexer spindexer;
    boolean wasFinished = false;
    Timer finishedTimer;
    double finishedTimeThreshold = 1000;

    public SpindexerSpinAngle(Spindexer spindexer, Angle angle, double power) {
        this.angle = angle;
        this.spindexer = spindexer;
        this.power = power;
        addRequirements(this.spindexer);
    }

    @Override
    public void initialize() {
        targetAngle = spindexer.getEncoder().getAngleUnnormalized() + angle.toDegrees();
        spindexer.spin(angle.sign() * power);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = spindexer.getEncoder().getAngleUnnormalized();
//        boolean there = Math.abs(targetAngle - currentAngle) < stopThreshold;
//        if (there) {
//            if (!wasFinished) finishedTimer.restart();
//            if (finishedTimer.getTime() < finishedTimeThreshold) {
//                spindexer.goToAngle(Angle.fromDegrees(targetAngle), CRServoEx2.RunMode.OptimizedPositionalControl);
//                return true;
//            }
//        }
//        wasFinished = there;
//        return there;
        return Math.abs(targetAngle - currentAngle) < stopThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.getTurner().stop();
    }
}
