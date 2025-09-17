package org.firstinspires.ftc.teamcode.OutakeTesting;

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
@TeleOp(name = "Flywheel")
public class FlywheelTesting extends OpMode {
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
    public static double power = 1;
    public static double diameter =  0.07;//in m

    @Override
    public void init() {
        toggle = new ButtonToggle();
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotorEx.Direction.FORWARD);
        right.setDirection(DcMotorEx.Direction.REVERSE);

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

    @Override
    public void loop() {
        timer.resetTimer();

        if(toggle.update(gamepad1.x)){
            resetEncoders();
        }

        left.setPower(power);
        right.setPower(power);

        double leftRpmNominal = left.getVelocity() / ticksPerRevolution * 60;
        double rightRpmNominal = right.getVelocity() / ticksPerRevolution * 60;
        double leftRpmActual = leftRpmNominal * fourToOneGR;
        double rightRpmActual = rightRpmNominal * fourToOneGR;

        double wheelCircumference = Math.PI * diameter;
        double leftExitVel = leftRpmActual / 60 * wheelCircumference;
        double rightExitVel = rightRpmActual / 60 * wheelCircumference;

        graph.addData("Left RPM Actual", leftRpmActual);
        graph.addData("Right RPM Actual", rightRpmActual);
        graph.addData("Left RPM Nominal", leftRpmNominal);
        graph.addData("Right RPM Nominal", rightRpmNominal);
        graph.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
        graph.addData("Power", power);
        graph.update();

        telemetry.addData("Left rpm nominal", leftRpmNominal);
        telemetry.addData("Right rpm nominal", rightRpmNominal);
        telemetry.addData("Left rpm actual", leftRpmActual);
        telemetry.addData("Right rpm actual", rightRpmActual);
        telemetry.addData("Update rate", 1/ timer.getLastUpdateTime());
        telemetry.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
        telemetry.addData("Power", power);

        panelsTelemetry.update(telemetry);
    }
}
