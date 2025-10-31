package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;

public class SpindexerSpinAngle extends CommandBase {
    public static double stopThreshold = 20;

    private double initialAngle, targetAngle;
    private final double power;
    private final Angle angle;
    private final Spindexer spindexer;

    public SpindexerSpinAngle(Spindexer spindexer, Angle angle, double power) {
        this.angle = angle;
        this.spindexer = spindexer;
        this.power = power;
        addRequirements(this.spindexer);
    }

    @Override
    public void initialize() {
        initialAngle = spindexer.getEncoder().getAngleUnnormalized();
        targetAngle = spindexer.getEncoder().getAngleUnnormalized() + angle.toDegrees();
        spindexer.spin(angle.sign() * power);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = spindexer.getEncoder().getAngleUnnormalized();
        return Math.abs(targetAngle - currentAngle) < stopThreshold ;
    }
}
