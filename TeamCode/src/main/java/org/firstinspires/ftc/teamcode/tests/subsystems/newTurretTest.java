//package org.firstinspires.ftc.teamcode.tests.subsystems;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//
//@TeleOp(name = "Turret Prototype Test", group = "Test")
//public class newTurretTest extends OpMode {
//
//    Turret turret;
//
//    // Odometry encoders
//    DcMotorEx leftOdo;
//    DcMotorEx rightOdo;
//
//    double targetHeading = 0;
//
//    // Tune this value for your robot
//    public static double TRACK_WIDTH_TICKS = 14000;
//
//    @Override
//    public void init() {
//
//        turret = znew Turret(hardwareMap);
//
//        leftOdo = hardwareMap.get(DcMotorEx.class, "leftOdo");
//        rightOdo = hardwareMap.get(DcMotorEx.class, "rightOdo");
//
//        leftOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftOdo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        rightOdo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    @Override
//    public void loop() {
//
//        // Adjust target heading with joystick
//        targetHeading += gamepad1.left_stick_x * 2;
//
//        // Clamp heading range
//        if (targetHeading > 180) targetHeading = 180;
//        if (targetHeading < -180) targetHeading = -180;
//
////        turret.setTargetHeading(targetHeading);
////
////        double robotHeading = getRobotHeading();
////
////        turret.update(robotHeading);
////
////        telemetry.addData("Target Heading", targetHeading);
////        telemetry.addData("Robot Heading", robotHeading);
////        telemetry.addData("Turret Angle", turret.getTurretAngle());
////        telemetry.addData("Left Odo", leftOdo.getCurrentPosition());
////        telemetry.addData("Right Odo", rightOdo.getCurrentPosition());
////        telemetry.update();
//    }
//
//    public double getRobotHeading() {
//
//        double left = leftOdo.getCurrentPosition();
//        double right = rightOdo.getCurrentPosition();
//
//        double headingRadians = (right - left) / TRACK_WIDTH_TICKS;
//
//        return Math.toDegrees(headingRadians);
//    }
//}