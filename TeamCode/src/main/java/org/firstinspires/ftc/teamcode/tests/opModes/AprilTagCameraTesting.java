package org.firstinspires.ftc.teamcode.tests.opModes;

import static org.firstinspires.ftc.teamcode.util.ExtraFns.normAngle;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag Camera Test")
@Config
@Configurable
public class AprilTagCameraTesting extends OpMode {

    // Telemetry telemetryM;

    public static double[] pidAutoAlign = new double[]{1.0, 0, 0.1};//1.5, 0, 0.1
    AprilTagWebcam arducam = new AprilTagWebcam();
    double prevHeadingError;
    double turnPower;
    public static boolean autoAlign = true;
    Pose currentPose;

    double targetheading;
    public static int TAG_ID = 21;
    Timer timer;
    Timer gameTimer;
    Follower follower;
    ShootSide shootSide = ShootSide.LEFT;
    AprilTagDetection tag;
    double cameraYawRelative;

    double cameraYawGlobal;
    double headingError;
    double rightAprilAngle = 38.565;//degrees
    double leftAprilAngle = 90 + 38.565;
    public static boolean useArducam = true;
    Timer arducamTimer;
    Pose leftTarget = new Pose(0, 144, Math.toRadians(45));
    Pose rightTarget = new Pose(144, 144, Math.toRadians(-45));
    Pose startPose = new Pose(72, 8, 0);
    double currSpeed = 1.0;
    double prevArducamTime = 0;
    public static double minArduTimeUpdate = 0;
    boolean triggerArducamDetection;
    WheelControl wheelControl;
    public static double swapToCameraThreshold = Math.toRadians(20);
    @Override
    public void init() {
        follower = ConstantsBot.createDoublePinpointFollower(hardwareMap);
        follower.setPose(startPose);
        wheelControl = new WheelControl(hardwareMap);
        arducam = new AprilTagWebcam();
        arducam.init(hardwareMap, ConfigNames.arducam, telemetry);
        arducamTimer = new Timer();
        timer = new Timer();
        gameTimer = new Timer();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        if(!autoAlign) {
            wheelControl.drive_relative(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x * currSpeed, currSpeed);
        } else{
            wheelControl.drive_relative(gamepad1.left_stick_y, gamepad1.left_stick_x, turnPower, currSpeed);
        }

        currentPose = follower.getPose();

        // camera fps and latency
        telemetry.addData("FPS", arducam.getFps());
        telemetry.addData("Latency (ms)", arducam.getLatencyMs());

//        if (tag != null) {
//            // position relative to camera (meters)
//            double x = arducam.aprilTagXPos(tag);
//            double z = arducam.aprilTagZPos(tag);
//
//            // horizontal angle to apriltag
//            double angleRad = Math.atan(x / z);
//            double angleDeg = Math.toDegrees(angleRad);
//
//            // straight line distance to april tag
//            double distance = Math.hypot(x, z);
//
//            telemetry.addData("Tag ID", tag.id);
//            telemetry.addData("X Offset (m)", "%.3f", x);
//            telemetry.addData("Z Distance (m)", "%.3f", z);
//            telemetry.addData("Total Distance (m)", "%.3f", distance);
//            telemetry.addData("Angle (deg)", "%.2f", angleDeg);
//
//            //pose, yaw, pitch
//
//            arducam.displayDetectionTelemetry(tag);
//
//        } else {
//            telemetry.addData("AprilTag " + TAG_ID, "Not Visible");
//        }

        telemetry.addData("Update Rate", 1000.0 / gameTimer.getDeltaTime());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Auto Align", autoAlign);
        telemetry.addData("Target Heading", convertRadToDegrees(targetheading));
        telemetry.addData("Heading Error(Alignment)", convertRadToDegrees(headingError));
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("Update Arducam", triggerArducamDetection);
        telemetry.addData("Camera Yaw Global", cameraYawGlobal);
        telemetry.addData("Camera Yaw Rel", cameraYawRelative);
        telemetry.addData("Tag", tag == null ? "NONE" : tag.id);

        setAlignTurnPower();


        telemetry.update();
    }

    private void setAlignTurnPower(){

        //MODIFY so that the heading is facing the outake side, not the intake side
        if(autoAlign) {

            Pose outakePose = new Pose(currentPose.getX(), currentPose.getY(), normAngle(currentPose.getHeading() + Math.PI));
            //add compensation for spindexer direction
//            double distToTarget = getDistance(follower.getPose(), outakePose);
//            double compY = outakePose.getY() + distToTarget * Math.tan(spindexerCompensationOffset);
            // double compY = outakePose.getY() + spindexerDirection * distToTarget * Math.tan(spindexerCompensationOffset);
//            Pose compensatedPose = new Pose(outakePose.getX(), outakePose.getY(), outakePose.getHeading());
//            if (spindexer.getTurner().getServo().getPower() < 0.1) {
//                compensatedPose = outakePose;
//            }

            if (useArducam && arducamTimer.getTime() - prevArducamTime >= minArduTimeUpdate) {
                triggerArducamDetection = true;
                prevArducamTime = arducamTimer.getTime();
                arducam.update();
                if (shootSide == ShootSide.LEFT) {
                    tag = arducam.getTagBySpecificId(20);
                } else {
                    tag = arducam.getTagBySpecificId(24);
                }
            } else{
                triggerArducamDetection = false;
            }

            headingError = getAngleError(outakePose, ((shootSide == ShootSide.LEFT) ? leftTarget : rightTarget), outakePose.getHeading());

            if (Math.abs(headingError) < swapToCameraThreshold && useArducam) {
                if (tag != null) {
                    cameraYawRelative = tag.ftcPose.yaw;
                    cameraYawGlobal = (cameraYawRelative + ((shootSide == ShootSide.LEFT) ? leftAprilAngle : rightAprilAngle)) + Math.PI;
                    headingError = getAngleError(outakePose, ((shootSide == ShootSide.LEFT) ? leftTarget : rightTarget), cameraYawGlobal);
                }
            }

            turnPower = calculateGamepadPID(prevHeadingError, headingError);
            prevHeadingError = headingError;
        }
    }

    public double getAngleError(Pose position, Pose target, double positionHeading){
        double deltaY = target.getY() - position.getY();
        double deltaX = target.getX() - position.getX();
        double heading = Math.atan2(deltaY, deltaX);
        heading = normAngle(heading);
        this.targetheading = heading;
        //heading is in absolute degrees
        double error = heading - positionHeading;
        double errorSign = (error > 0 ) ? -1 : 1;
        if(Math.abs(error) > Math.PI){
            error = errorSign * (2 * Math.PI - Math.abs(positionHeading - heading));
        }

        error = normAnglePlusMinus2PI(error);
        return error;
    }
    private double convertRadToDegrees(double val){
        return val * 180 / Math.PI;
    }

    private double normAnglePlusMinus2PI(double error){
        while(error < -Math.PI *2){
            error += Math.PI *2;
        }
        while(error > Math.PI * 2){
            error -= Math.PI * 2;
        }
        return error;
    }

    private double calculateGamepadPID(double prevHeadingError, double headingError){
//        double filteredHeadingError = (1-alignmentWeight) * headingError + alignmentWeight * prevHeadingError;
        double filteredHeadingError = headingError;
        double pGain = pidAutoAlign[0] * filteredHeadingError;
        double dGain = pidAutoAlign[2] * (filteredHeadingError - prevHeadingError) / timer.getDeltaTime();

        double power = pGain + dGain;
        power = Math.max(-1, Math.min(1, power));


//          if(Math.abs(power) <= minPowerHeadingAlign) {
//            power = 0;
//        }

        if (Math.abs(filteredHeadingError) > Math.toRadians(1.5)) {
            power += Math.signum(filteredHeadingError) * 0.06;
        } else{
            power = 0;
        }


        return power;
    }


}
