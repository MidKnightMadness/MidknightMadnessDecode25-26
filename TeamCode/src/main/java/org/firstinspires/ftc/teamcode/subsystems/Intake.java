package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.MathFuncs;

import java.util.Map;



@Config
@Configurable
public class Intake extends SubsystemBase {
    DcMotorEx intakeMotorLeft;
    DcMotorEx intakeMotorRight;

    public static double reference_voltage = 12.5;
    public static boolean leftMotorDirectionForward = false;
    public static boolean rightMotorDirectionForward = false;


    public enum RunMode{
        VelocityControl, RawPower
    }
    RunMode runMode;
    double motorGearRatio = 5.31;
    public static Map<Double, Double> grToMultiplier = Map.of(
            3., 2.89,
            4., 3.61,
            5., 5.23
    ); // Unused for now


    public Intake(HardwareMap hardwareMap, RunMode runMode){
        intakeMotorLeft = hardwareMap.get(DcMotorEx.class, ConfigNames.intakeMotorLeft);
        intakeMotorRight = hardwareMap.get(DcMotorEx.class, ConfigNames.intakeMotorRight);

        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorLeft.setDirection(leftMotorDirectionForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        intakeMotorRight.setDirection(rightMotorDirectionForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
    }


    public DcMotorEx getLeftMotor(){
        return intakeMotorLeft;
    }

    public DcMotorEx getRightMotor() {
        return intakeMotorRight;
    }


    public void setDirectPower(double power){
        double motorPower = MathFuncs.clampOutput(power, -1, 1);
        intakeMotorLeft.setPower(motorPower);
        intakeMotorRight.setPower(motorPower);
    }

    public void setDirectPower(double power, double currVolt){
        double motorPower = MathFuncs.clampOutput(power, -1, 1);
        intakeMotorLeft.setPower(motorPower * reference_voltage / currVolt);
    }


    public void stopPower(){
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
    }

    public double getMotorVelocity(){
        return intakeMotorLeft.getVelocity(AngleUnit.RADIANS);
    }
}
