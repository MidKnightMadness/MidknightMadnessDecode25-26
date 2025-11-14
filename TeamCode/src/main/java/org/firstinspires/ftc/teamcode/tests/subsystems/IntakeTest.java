package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@Configurable
@TeleOp(name = "Intake Test", group = "Intake")
public class IntakeTest extends OpMode {
    Intake intake;
    public static Intake.RunMode runMode = Intake.RunMode.RawPower;
    public static double motorPower = 1;
    public static double targetVelocity = 1000;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;
    public static double kA = 0;
    public static double kV = 0;

    @Override
    public void init() {
        telemetry.addLine("Mode 0 : RawPower, Mode 1: VelocityControl");
        intake = new Intake(hardwareMap, runMode);
        if(runMode == Intake.RunMode.VelocityControl){
            intake.setPid(kP, kI, kD);
            intake.setFeedforward(kS, kV, kA);
        }
    }

    @Override
    public void loop() {
        if(runMode == Intake.RunMode.RawPower){
            intake.setDirectPower(motorPower);
        }
        else{
            intake.setVelocity(targetVelocity);
        }

        telemetry.addData("RunMode", runMode);
        telemetry.addData("Current Power", motorPower);
        telemetry.addData("Motor Velocity", intake.getMotorVelocity());
        telemetry.addData("Target Velocity", targetVelocity);

    }
}
