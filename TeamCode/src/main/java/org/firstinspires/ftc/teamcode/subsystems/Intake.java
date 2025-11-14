package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Direction;

import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Math;

import java.util.Map;



@Config
@Configurable
public class Intake extends SubsystemBase {
    MotorEx intakeMotor;

    public static boolean motorDirectionForward = true;

    public enum RunMode{
        VelocityControl, RawPower
    }
    RunMode runMode;
    double motorGearRatio = 3;
    public static Map<Double, Double> grToMultiplier = Map.of(
            3., 2.89,
            4., 3.61,
            5., 5.23
    ); // Unused for now


    public Intake(HardwareMap hardwareMap, RunMode runMode){
        intakeMotor = new MotorEx(hardwareMap, ConfigNames.intakeMotor);
        setRunMode(runMode);
        intakeMotor.motor.setDirection(motorDirectionForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
    }

    public void setPid(double kp, double ki, double kd) {
        intakeMotor.setVeloCoefficients(kp, ki, kd);
    }
    public void setFeedforward(double kS, double kV, double kA){
        intakeMotor.setFeedforwardCoefficients(kS, kV, kA);
    }
    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
        if(runMode == RunMode.RawPower){
            intakeMotor.setRunMode(Motor.RunMode.RawPower);
        }
        else{
            intakeMotor.setRunMode(Motor.RunMode.VelocityControl);
        }
    }

    public void setDirectPower(double power){
        double motorPower = Math.clampOutput(power, 0, 1);
        intakeMotor.motor.setPower(motorPower);
    }

    public void setVelocity(double vel){
        double correctedVelocity = vel * grToMultiplier.get(motorGearRatio);
        if(runMode == RunMode.VelocityControl){
            intakeMotor.set(correctedVelocity);
        }
        else{
            intakeMotor.set(correctedVelocity / intakeMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
        }
    }

    public void stopPower(){
        intakeMotor.motor.setPower(0);
    }
    public void resetEncoder(){
        intakeMotor.encoder.reset();
    }

    public double getMotorVelocity(){
        return intakeMotor.getCorrectedVelocity();
    }
}
