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
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Util.ButtonToggle;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Configurable
@TeleOp(name = "Flywheel", group = "Experiments")
public class DoubleFlywheelTest extends OpMode {
    DcMotorEx left;
    DcMotorEx right;

    //nominal to actual gear ratios
    final double threeToOneGR = 2.89;
    final double fourToOneGR = 3.61;
    final double fiveToOneGR = 5.23;

    ButtonToggle toggle;
    GraphManager graph;
    TelemetryManager panelsTelemetry;


    //from motor specs rev
    final double ticksPerRevolution = 28;


    Timer timer;
    //public static double power = 1;
    //Replaced power with rpm

    //change left and right rpm to add spin, can edit through FTC panels under configurables
    public static double leftRpm = 1200;
    public static double rightRpm = 1200;


    //diameter of the wheel to calculate approximate exit velocity(not needed right now)
    public static double wheelDiameter =  0.07;//in m

    //PIDF gains
    public static double pGain = 0.03;//Really only need p and maybe f gains
    public static double iGain = 0;
    public static double dGain = 0;
    public static double fGain = 0;

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
        left.setVelocityPIDFCoefficients(pGain, iGain, dGain, fGain);
        right.setVelocityPIDFCoefficients(pGain, iGain, dGain, fGain);
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

        //Set the motors to the target velocities from the target RPMs
        double targetLeftRPMNom = leftRpm * fourToOneGR;
        double targetLeftVel = targetLeftRPMNom / 60 * ticksPerRevolution;//ticks per revolution

        double targetRightRPMNom = rightRpm * fourToOneGR;
        double targetRightVel = targetRightRPMNom / 60 * ticksPerRevolution;//ticks per revolution

        left.setVelocity(targetLeftVel);
        right.setVelocity(targetRightVel);

//        left.setPower(power);
//        right.setPower(power);

        double leftRpmNominal = left.getVelocity() / ticksPerRevolution * 60;
        double rightRpmNominal = right.getVelocity() / ticksPerRevolution * 60;
        double leftRpmActual = leftRpmNominal / fourToOneGR;
        double rightRpmActual = rightRpmNominal / fourToOneGR;

        double wheelCircumference = Math.PI * wheelDiameter;
        double leftExitVel = leftRpmActual / 60 * wheelCircumference;
        double rightExitVel = rightRpmActual / 60 * wheelCircumference;


        //Display data on Ftc Panels: navigate to 192.168.43.1:8001 on robot wifi
        graph.addData("Left RPM Actual", leftRpmActual);
        graph.addData("Right RPM Actual", rightRpmActual);
        graph.addData("Left RPM Nominal", leftRpmNominal);
        graph.addData("Right RPM Nominal", rightRpmNominal);
        graph.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
//        graph.addData("Power", power);
        graph.update();

        panelsTelemetry.addData("Left rpm nominal", leftRpmNominal);
        panelsTelemetry.addData("Right rpm nominal", rightRpmNominal);
        panelsTelemetry.addData("Left rpm actual", leftRpmActual);
        panelsTelemetry.addData("Right rpm actual", rightRpmActual);
        panelsTelemetry.addData("Update rate", 1/ timer.getLastUpdateTime());
        panelsTelemetry.addData("Average Exit Velocity", (leftExitVel + rightExitVel) / 2);
//        panelsTelemetry.addData("Power", power);

        panelsTelemetry.update(telemetry);
    }
}
