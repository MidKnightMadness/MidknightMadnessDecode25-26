package org.firstinspires.ftc.teamcode.Outake;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.ButtonToggle;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Configurable
@TeleOp(name = "2Flywheel", group = "Experiments")
public class DoubleFlywheel extends OpMode {
    DcMotorEx left;
    DcMotorEx right;
    //nominal to actual gear ratios
    final double threeToOneGR = 2.89;
    final double fourToOneGR = 3.61;
    final double fiveToOneGR = 5.23;

    ButtonToggle toggle;
    GraphManager graph;
    TelemetryManager panelsTelemetry;


    final double ticksPerRevolution = 28;

    Timer timer;
    //Direct Power Constants
    public static double leftPower = 1;
    public static double rightPower = 1;
    public static double diameter =  0.07;//in m

    public static boolean leftForward = true;
    public static boolean rightForward = false;
    public static int mode = 1;//1 = power, 2 = custom PID

    //Custom PID Constants
    //Velocity and RPM Update terms

    double leftRpmNominal = 0;
    double rightRpmNominal = 0;
    double leftRpmActual = 0;
    double rightRpmActual = 0;
    double leftExitVel = 0;
    double rightExitVel = 0;
    double wheelCircumference = Math.PI * diameter;


    //Target Velocity Gains
    public static double targetLeftRpm = 1700;
    public static double targetRightRpm = 1700;
    public static double maxRPM = 2100;

    //PIDF gains
    public static double pGain = 0.03;
    public static double iGain = 0;
    public static double dGain = 0;
    public static double fGain = 1 / maxRPM;

    //PIDF values
    double leftIntegralComponent = 0;
    double rightIntegralComponent = 0;
    double prevLeftError = 0;
    double prevRightError = 0;



    @Override
    public void init() {
        toggle = new ButtonToggle();
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        telemetry.addLine("Change Mode to 1 for Power and 2 for Custom PID");
        telemetry.addLine("Program Initalized");
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



    public void updateVelocity(){
         leftRpmNominal = left.getVelocity() / ticksPerRevolution * 60;
         rightRpmNominal = right.getVelocity() / ticksPerRevolution * 60;
         leftRpmActual = leftRpmNominal / threeToOneGR;
         rightRpmActual = rightRpmNominal / threeToOneGR;

         wheelCircumference = Math.PI * diameter;
         leftExitVel = leftRpmActual / 60 * wheelCircumference;
         rightExitVel = rightRpmActual / 60 * wheelCircumference;
    }

    //PIDF values
    double currLeftRpm, currRightRpm;
    double leftError, rightError;
    double deltaTime;
    double leftDerivative, rightDerivative;
    double leftOutput, rightOutput;
    @Override
    public void loop() {
        timer.updateTime();

        if(toggle.update(gamepad1.x)){
            resetEncoders();
        }


        if(mode == 2){
            currLeftRpm = left.getVelocity() / ticksPerRevolution * 60 / threeToOneGR;
            currRightRpm = right.getVelocity() / ticksPerRevolution * 60 / threeToOneGR;

            //PIDF logic
            leftError = targetLeftRpm - currLeftRpm;
            rightError = targetRightRpm - currRightRpm;

            deltaTime = timer.getDeltaTime();

            leftIntegralComponent += leftError * deltaTime;
            rightIntegralComponent += rightError * deltaTime;

            leftDerivative = (leftError - prevLeftError) / deltaTime;
            rightDerivative = (rightError - prevRightError) / deltaTime;

            leftOutput = pGain * leftError + iGain * leftIntegralComponent + dGain * leftDerivative + fGain * targetLeftRpm;
            rightOutput = pGain * rightError + iGain * rightIntegralComponent + dGain * rightDerivative + fGain * targetRightRpm;

            leftPower = (leftOutput <= 1 &&  leftOutput>=-1) ? leftOutput : leftOutput > 1 ? 1 : -1;
            rightPower = (rightOutput <= 1 &&  rightOutput>=-1) ? rightOutput : rightOutput > 1 ? 1 : -1;
        }

        updateMotorPowers(leftPower, rightPower);
        updateVelocity();
        updatePanelsGraph();
        updatePanelsTelemetry();
        updateTelemetry();



    }
    private void updateMotorPowers(double leftPower, double rightPower) {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    private void updateTelemetry(){
        if(mode == 2) {
            telemetry.addData("Left Error", leftError);
            telemetry.addData("Right Error", rightError);
            telemetry.addData("Left Integral", leftIntegralComponent);
            telemetry.addData("Right Integral", rightIntegralComponent);
            telemetry.addData("Left Derivative", leftDerivative);
            telemetry.addData("Right Derivative", rightDerivative);
            telemetry.addData("Left Output", leftOutput);
            telemetry.addData("Right Output", rightOutput);
        }

        telemetry.addData("Left RPM Nominal", leftRpmNominal);
        telemetry.addData("Right RPM Nominal", rightRpmNominal);
        telemetry.addData("Left RPM Actual", leftRpmActual);
        telemetry.addData("Right RPM Actual", rightRpmActual);

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
        telemetry.addData("Update rate", 1/ deltaTime);
    }
    private void updatePanelsTelemetry() {
        if(mode == 2) {
            panelsTelemetry.addData("Left Error", leftError);
            panelsTelemetry.addData("Right Error", rightError);
            panelsTelemetry.addData("Left Integral", leftIntegralComponent);
            panelsTelemetry.addData("Right Integral", rightIntegralComponent);
            panelsTelemetry.addData("Left Derivative", leftDerivative);
            panelsTelemetry.addData("Right Derivative", rightDerivative);
            panelsTelemetry.addData("Left Output", leftOutput);
            panelsTelemetry.addData("Right Output", rightOutput);
        }

        panelsTelemetry.addData("Left RPM Nominal", leftRpmNominal);
        panelsTelemetry.addData("Right RPM Nominal", rightRpmNominal);
        panelsTelemetry.addData("Left RPM Actual", leftRpmActual);
        panelsTelemetry.addData("Right RPM Actual", rightRpmActual);

        panelsTelemetry.addData("Left Power", leftPower);
        panelsTelemetry.addData("Right Power", rightPower);
        panelsTelemetry.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
        panelsTelemetry.addData("Update rate", 1/ deltaTime);

        panelsTelemetry.update(telemetry);

    }

    public void updatePanelsGraph(){
        if(mode == 2) {
            graph.addData("Left Error", leftError);
            graph.addData("Right Error", rightError);
            graph.addData("Left Integral", leftIntegralComponent);
            graph.addData("Right Integral", rightIntegralComponent);
            graph.addData("Left Derivative", leftDerivative);
            graph.addData("Right Derivative", rightDerivative);
            graph.addData("Left Output", leftOutput);
            graph.addData("Right Output", rightOutput);
        }
        graph.addData("Left RPM Nominal", leftRpmNominal);
        graph.addData("Right RPM Nominal", rightRpmNominal);
        graph.addData("Left RPM Actual", leftRpmActual);
        graph.addData("Right RPM Actual", rightRpmActual);

        graph.addData("Left Power", leftPower);
        graph.addData("Right Power", rightPower);
        graph.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
        graph.addData("Update rate", 1/ deltaTime);

        graph.update();
    }


}