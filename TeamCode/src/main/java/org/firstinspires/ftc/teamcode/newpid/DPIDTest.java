package org.firstinspires.ftc.teamcode.newpid;

import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.newpid.interpolators.NearestPointInterpolator;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@TeleOp
@Disabled
public class DPIDTest extends OpMode {
    public static double kp = 0.01, ki = 0, kd = 0.001;
    public static double target = 1000;
    public static String motorName = "FL";
    DPIDController<NearestPointInterpolator> controller;
    TelemetryManager telemetryM;
    GraphManager graphM;
    DcMotorEx motor;
    Timer timer;
    Buffer buffer;

    @Override
    public void init() {
        controller = new DPIDController<>(
                NearestPointInterpolator::new, kp, ki, kd
        );
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        target = motor.getCurrentPosition() + 10000;
        timer = new Timer(TimeUnit.MILLISECONDS);
        buffer = new Buffer(10, 0.01);
    }

    @Override
    public void loop() {
        double loopTime = timer.getDeltaTime();
        double position = motor.getCurrentPosition();
        double velocity = motor.getVelocity();
        double acceleration = buffer.getDerivative();
        double power = controller.calculate(position, target);
        motor.setPower(power);
        buffer.add(velocity, timer.getTime());

        telemetryM.addData("loop time", loopTime);
        telemetryM.addData("position", position);
        telemetryM.addData("velocity", velocity);
        telemetryM.addData("accel", acceleration);
        telemetryM.addData("target", target);
        telemetryM.addData("error", position-target);
        graphM.addData("error", position-target);
        graphM.addData("loop time", loopTime);

        telemetryM.update(telemetry);
        graphM.update();
    }
}
