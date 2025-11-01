package org.firstinspires.ftc.teamcode.old.opModes;

import static java.lang.Math.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;


@Configurable
@Config
@TeleOp(name = "2FlywheelPID", group = "Experiments")
public class DoubleFlywheelCustomPID extends OpMode {
    DcMotorEx left;
    DcMotorEx right;

    //nominal to actual gear ratios
    final double threeToOneGR = 2.89;
    final double fourToOneGR = 3.61;
    final double fiveToOneGR = 5.23;

    ButtonToggle toggle;
    GraphManager graph;
    TelemetryManager panelsTelemetry;


    //motor constants
    final double ticksPerRevolution = 28;
    public static double wheelDiameter =  0.07;//in m


    Timer timer;

    //change left and right rpm to add spin, can edit through FTC panels under configurables
    public static double targetLeftRpm = 1200;
    public static double targetRightRpm = 900;



    public static double maxRPM = 2100;
    //PIDF gains
    public static double pGain = 0.03;//Really only need p and maybe f gains
    public static double iGain = 0;
    public static double dGain = 0;
    public static double fGain = 1/ maxRPM;

    public static boolean leftForward = true;
    public static boolean rightForward = false;


    @Override
    public void init() {
        toggle = new ButtonToggle();
        left = hardwareMap.get(DcMotorEx.class, ConfigNames.lowFlywheelMotor);
        right = hardwareMap.get(DcMotorEx.class, ConfigNames.highFlywheelMotor);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(leftForward ? DcMotorEx.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        right.setDirection(rightForward ? DcMotorEx.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);


        graph = PanelsGraph.INSTANCE.getManager();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        timer = new Timer();

    }


    public void resetEncoders(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    double leftIntegralComponent = 0;
    double rightIntegralComponent = 0;
    double prevLeftError = 0;
    double prevRightError = 0;

    @Override
    public void loop() {
        if (toggle.update(gamepad1.x)){
            resetEncoders();
        }

        //Assuming we want different speeds for the flywheel
        //Get current rpms
        double currLeftRpm = left.getVelocity() / ticksPerRevolution * 60 / threeToOneGR;
        double currRightRpm = right.getVelocity() / ticksPerRevolution * 60 / threeToOneGR;

        //PIDF logic
        double leftError = targetLeftRpm - currLeftRpm;
        double rightError = targetRightRpm - currRightRpm;

        double deltaTime = timer.getDeltaTime();

        leftIntegralComponent += leftError * deltaTime;
        rightIntegralComponent += rightError * deltaTime;

        double leftDerivative = (leftError - prevLeftError) / deltaTime;
        double rightDerivative = (rightError - prevRightError) / deltaTime;

        double leftOutput = pGain * leftError + iGain * leftIntegralComponent + dGain * leftDerivative + fGain * targetLeftRpm;
        double rightOutput = pGain * rightError + iGain * rightIntegralComponent + dGain * rightDerivative + fGain * targetRightRpm;

        double leftPower = (leftOutput <= 1 &&  leftOutput>=-1) ? leftOutput : leftOutput > 1 ? 1 : -1;
        double rightPower = (rightOutput <= 1 &&  rightOutput>=-1) ? rightOutput : rightOutput > 1 ? 1 : -1;

        left.setPower(leftPower);
        right.setPower(rightPower);

        telemetry.addData("Update Rate", 1/ deltaTime);
        telemetry.addData("Target Left", targetLeftRpm);
        telemetry.addData("Target Right", targetRightRpm);

        telemetry.addData("Left Vel:", currLeftRpm);
        telemetry.addData("Right Vel:", currRightRpm);

        double wheelCircumference = Math.PI * wheelDiameter;
        double leftExitVel = currLeftRpm / 60 * wheelCircumference;
        double rightExitVel = currRightRpm / 60 * wheelCircumference;


        //Display data on Ftc Panels: navigate to 192.168.43.1:8001 on robot wifi
        graph.addData("Left Current RPM", currLeftRpm);
        graph.addData("Right Current RPM", currRightRpm);


        //PID values
        graph.addData("Left Error", leftError);
        graph.addData("Right Error", rightError);
        graph.addData("Left Integral", leftIntegralComponent);
        graph.addData("Right Integral", rightIntegralComponent);
        graph.addData("Left Derivative", leftDerivative);
        graph.addData("Right Derivative", rightDerivative);
        graph.addData("Left Output", leftOutput);
        graph.addData("Right Output", rightOutput);

        graph.addData("Update rate", 1/ deltaTime);
        graph.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
        graph.update();


        panelsTelemetry.addData("Left Target RPM", targetLeftRpm);
        panelsTelemetry.addData("Left Current RPM", currLeftRpm);
        panelsTelemetry.addData("Right Target RPM", targetRightRpm);
        panelsTelemetry.addData("Right Current RPM", currRightRpm);

        //PID values
        panelsTelemetry.addData("Left Error", leftError);
        panelsTelemetry.addData("Right Error", rightError);
        panelsTelemetry.addData("Left Integral", leftIntegralComponent);
        panelsTelemetry.addData("Right Integral", rightIntegralComponent);
        panelsTelemetry.addData("Left Derivative", leftDerivative);
        panelsTelemetry.addData("Right Derivative", rightDerivative);
        panelsTelemetry.addData("Left Output", leftOutput);
        panelsTelemetry.addData("Right Output", rightOutput);

        panelsTelemetry.addData("Update rate", 1/ deltaTime);
        panelsTelemetry.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);

        panelsTelemetry.update(telemetry);

        prevLeftError = leftError;
        prevRightError = rightError;
    }
}