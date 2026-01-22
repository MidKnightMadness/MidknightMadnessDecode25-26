package org.firstinspires.ftc.teamcode.tests.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Turret Test")
public class TurretTest extends OpMode {

    DcMotor turretMotor;

    // april tag
    int idNum = 20;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // encoder constants
    static final double MOTOR_TICKS = 28.0;           // ticks per motor revolution
    static final double THREE_TO_ONE = 3.0;            // 3:1 reduction
    static final double TURRET_GEAR = 130.0 / 30.0;    // 30T -> 130T

    static final double TOTAL_GEAR_RATIO = THREE_TO_ONE * TURRET_GEAR;

    static final double TICKS_PER_RADIAN =
            (MOTOR_TICKS * TOTAL_GEAR_RATIO) / (2 * Math.PI);

    // controls
    static final double MOTOR_POWER = 0.3;
    static final double ANGLE_DEADBAND_RAD = Math.toRadians(1.0);

    // joystick override
    static final double JOYSTICK_DEADBAND = 0.15;
    static final double MANUAL_TURN_POWER = 0.35; //tune for joystick sensitivity

    // soft limits (tune)
    static final int MIN_TICKS = -1400;
    static final int MAX_TICKS = 1400;

    int lastTargetTicks = 0;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MOTOR_POWER);

        aprilTagWebcam.init(hardwareMap, ConfigNames.arducam, telemetry);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        int currentTicks = turretMotor.getCurrentPosition();

        //manual override
        double joystick = gamepad1.right_stick_x;

        if (Math.abs(joystick) > JOYSTICK_DEADBAND) {

            // switch to manual control
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double power = joystick * MANUAL_TURN_POWER;

            // soft limits (tune if they cause issues)
            if ((currentTicks <= MIN_TICKS && power < 0) ||
                    (currentTicks >= MAX_TICKS && power > 0)) {
                power = 0;
            }

            turretMotor.setPower(power);

            telemetry.addLine("Mode: MANUAL");
        }
        else {
           //auto aim with april tags by camera
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            aprilTagWebcam.update();
            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(idNum);

            if (tag != null) {

                double angleDeg = tag.ftcPose.bearing;
                double angleRad = Math.toRadians(angleDeg);

                if (Math.abs(angleRad) > ANGLE_DEADBAND_RAD) {

                    int targetTicks = (int) (angleRad * TICKS_PER_RADIAN);

                    // clamp to soft limits (also tune)
                    targetTicks = Math.max(MIN_TICKS,
                            Math.min(MAX_TICKS, targetTicks));

                    if (targetTicks != lastTargetTicks) {
                        turretMotor.setTargetPosition(targetTicks);
                        turretMotor.setPower(MOTOR_POWER);
                        lastTargetTicks = targetTicks;
                    }
                }

                telemetry.addLine("Mode: AUTO");
                telemetry.addData("Apriltag ID", tag.id);
                telemetry.addData("Angle (deg)", angleDeg);
                telemetry.addData("Target Ticks", lastTargetTicks);

            } else {
                telemetry.addLine("Mode: AUTO");
                telemetry.addData("AprilTag", "Not Visible");
            }
        }

        telemetry.addData("Motor Position", currentTicks);
    }
}
