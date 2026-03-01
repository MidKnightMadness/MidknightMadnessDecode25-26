package org.firstinspires.ftc.teamcode.newpid;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@TeleOp
@Configurable
@Disabled
public class StandardPIDTest extends OpMode {
    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0.001;
    public static String motorName = "FL";
    public static double target = 10000;
    private double trueTarget;
    PIDController controller;
    TelemetryManager telemetryM;
    GraphManager graphM;
    DcMotorEx motor;
    Timer timer;
    Buffer buffer;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        controller = new PIDController(kp, ki, kd);
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        timer = new Timer(TimeUnit.MILLISECONDS);
        buffer = new Buffer(10, 0.01);
        trueTarget = motor.getCurrentPosition() + target;
    }

    @Override
    public void loop() {
        double loopTime = timer.getDeltaTime(TimeUnit.MILLISECONDS);
        double position = motor.getCurrentPosition();
        double velocity = motor.getVelocity();
        double acceleration = buffer.getDerivative();
        double power = controller.calculate(position, trueTarget);
        motor.setPower(power);
        buffer.add(velocity, timer.getTime());

        telemetryM.addData("loop time", loopTime);
        telemetryM.addData("position", position);
        telemetryM.addData("velocity", velocity);
        telemetryM.addData("accel", acceleration);
        telemetryM.addData("target", trueTarget);
        telemetryM.addData("error", position-trueTarget);
        graphM.addData("error", position-trueTarget);
        graphM.addData("loop time", loopTime);

        telemetryM.update(telemetry);
        graphM.update();
    }
}
