package org.firstinspires.ftc.teamcode.newpid;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.newpid.interpolators.DelaunayTriangulationInterpolator;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

@TeleOp
public class DanielPIDTuner extends OpMode {
    public ArrayList<Double[]> velAndErrors;
    public ArrayList<Point3D> allData;
    public double vel, maxVel, maxVelTime;
    public int trial = 0;
    public State state;
    public PIDController controller;
    public double target;
    public DoubleSupplier stateSupplier;
    public DoubleSupplier derivSupplier;
    public String motorName = "FL";
    Buffer buffer;
    DcMotorEx motor;
    ArrayList<Double> times;
    Timer timer;
    TelemetryManager telemetryM;

    public enum State {
        findMaxVel,
        wait,
        measure,
        stop,
    }

    @Override
    public void init() {
        vel = 0;
        maxVel = 0;
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        buffer = new Buffer(10, 50);
        times = new ArrayList<>();
        state = State.findMaxVel;
        timer = new Timer();
        controller = new PIDController(0.01, 0, 0.001);
        allData = new ArrayList<>();
        stateSupplier = motor::getCurrentPosition;
        derivSupplier = motor::getVelocity;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        timer.restart();
    }

    @Override
    public void loop() {
        switch (state) {
            case findMaxVel:
                if (buffer.isStable()) {
                    maxVelTime = timer.getTime();
                    buffer.clear();
                    timer.restart();
                    motor.setPower(0);
                    for (double t = 50; t <= 1000; t += 50) {
                        times.add(t);
                    }
                    state = State.wait;
                    break;
                }
                buffer.add(derivSupplier.getAsDouble(), timer.getTime());
                motor.setPower(-1);
                break;

            case wait:
                if (timer.getTime() > 2000) {
                    timer.restart();
                    target = stateSupplier.getAsDouble();
                    velAndErrors = new ArrayList<>();
                    state = State.measure;
                }
                break;

            case measure:
                if (trial >= times.size()) {
                    state = State.stop;
//                    DelaunayTriangulationInterpolator dt = new DelaunayTriangulationInterpolator(allData);
//                    try { dt.toFile("tests/predictError.json"); }
//                    catch (Exception e) {}
                    break;
                }
                if (buffer.isStable()) {
                    trial += 1;
                    buffer.clear();
                    timer.restart();
                    for (Double[] velAndError : velAndErrors) {
                        allData.add(new Point3D(
                                velAndError[0],
                                velAndError[1],
                                target - stateSupplier.getAsDouble()
                        ));
                    }
                    velAndErrors = new ArrayList<>();
                    break;
                }
                if (timer.getTime() < times.get(trial)) {
                    motor.setPower(-1);
                    break;
                }
                double currentVel = derivSupplier.getAsDouble();
                double currentPos = stateSupplier.getAsDouble();
                double power = controller.calculate(currentPos, target);
                velAndErrors.add(new Double[]{ currentVel, target - currentPos });
                buffer.add(currentVel, timer.getTime());
                motor.setPower(power);
                break;

            case stop:
                telemetryM.addData("All points", allData);
                break;
        }

        telemetryM.update(telemetry);
    }
}
