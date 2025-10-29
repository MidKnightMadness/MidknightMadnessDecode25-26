package org.firstinspires.ftc.teamcode.localization.kalmanFilter;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.DashboardDrawing;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
public class KalmanPinpointAprilLocalizer implements Localizer {
    KalmanPinpointAprilConstants constants;
    Telemetry telemetry;
    private Pose startPose;
    private Pose currentMergedPose = new Pose();
    private Pose aprilTagPose = new Pose();
    private Pose pinpointPose = new Pose();
    private Pose previousMergedPose = new Pose();
    HardwareMap hardwareMap;
    IMU imu;
    GoBildaPinpointDriver pinpoint;
    KalmanFilter kalmanFilter;
    Limelight3A limelight;
    boolean motifDetecting = false;
    boolean aprilTagDetected = false;
    private double previousHeading;
    private double totalHeading;
    private Pose currentVelocity;
    public static double leftThresholdDeg = 100;
    public static double rightThresholdDeg = 80;
    Timer timer;
    public KalmanPinpointAprilLocalizer(HardwareMap hardwareMap, KalmanPinpointAprilConstants lConstants, Telemetry telemetry){
        this(hardwareMap, lConstants, new Pose(), telemetry);
    }

    public KalmanPinpointAprilLocalizer(HardwareMap hardwareMap, KalmanPinpointAprilConstants lConstants, Pose startPose, Telemetry telemetry) {

        this.startPose = startPose;
        this.constants = lConstants;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //initalize imu
        imu = hardwareMap.get(IMU.class, lConstants.IMU_NAME);
        imu.initialize(new IMU.Parameters(lConstants.imuOrientation));

        initializePinpoint();
        initializeAprilTag();
        initializeKalman();
        timer = new Timer();
        previousHeading = startPose.getHeading();
        previousMergedPose = startPose;
        currentMergedPose = startPose;

    }

    private void initializeKalman(){
        kalmanFilter = new KalmanFilter(constants.Q, constants.R);
    }
    private void initializeAprilTag(){
        limelight = hardwareMap.get(Limelight3A.class, constants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(constants.startPipeline);//set it to motif pipeline
        motifDetecting = constants.MOTIF_DETECTION;
        if(!limelight.isRunning()){
            limelight.start();
            limelight.pipelineSwitch(constants.startPipeline);
        }
        //add complementary filter later or some other filter maybe
    }

    private void initializePinpoint(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, constants.PINPOINT_NAME);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startPose.getY(), startPose.getX(),AngleUnit.RADIANS, AngleUnit.normalizeRadians(startPose.getHeading()-Math.PI/2)));
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(constants.xDir, constants.yDir);
        pinpoint.setOffsets(constants.xOffset, constants.yOffset, constants.distUnit);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

//    private Pose offsetToBackLeftOrigin(Pose pose) {//wraps the angle to 0 -360 degrees per pedro
//        double x =  pose.getY() + 48;//reverse so positive = forward
//        double y =  pose.getX() + 48;
//        //transform from center y-x plane to traditional x-y plane from back corner left
//        double heading = pose.getHeading();
//        heading = wrapAngleRad(heading);
//        return new Pose(x, y, heading);
//    }

    @Override
    public void update() {
        if(!limelight.isRunning()){//if the limelight is not currently running a pipeline, we start running it
            limelight.start();
        }
        if(limelight.isRunning()) {
            if ((currentMergedPose.getHeading()> Math.toRadians(leftThresholdDeg))){
                limelight.pipelineSwitch(constants.leftPipelineNum);
            } else if(currentMergedPose.getHeading() < Math.toRadians(rightThresholdDeg)){
                limelight.pipelineSwitch(constants.rightPipelineNum);
            }
            //otherwise don't swap
        }

        updatePinpointPose();
        updateAprilTagPose();
        updateKalmanFilter();
        updateVelocityPose();
        updateTotalYaw();

        telemetry.addData("Start Pose: ", startPose != null ? startPose : 0);
        telemetry.addData("Pinpoint Pose: ", pinpointPose != null ? pinpointPose : 0);
        telemetry.addData("AprilTag Pose: ", aprilTagPose != null ? aprilTagPose : 0);
        telemetry.addData("Kalman Pose: ", currentMergedPose != null ? currentMergedPose : 0);
        telemetry.addData("Motif Detecting", motifDetecting);
        telemetry.addData("April Tag Detected", aprilTagDetected);

        drawPosesDashboard();
        telemetry.update();
        previousHeading = currentMergedPose.getHeading();
        previousMergedPose = currentMergedPose;

    }

    private void drawIndividualPose(Pose pose, String color){
        DashboardDrawing.drawRobot(new Pose(pose.getX(), pose.getY(), pose.getHeading()), color);
    }

    private void drawPosesDashboard() {
        drawIndividualPose(pinpointPose, "#FF0000");
        drawIndividualPose(aprilTagPose, "#FFFF00");
        drawIndividualPose(currentMergedPose, "#000000");
        DashboardDrawing.sendPacket();
    }

    public Pose updatePinpointPose(){
        pinpoint.update();
        pinpointPose = new Pose(72- pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getPosX(DistanceUnit.INCH) + startPose.getY(), normalizeAngleRad(pinpoint.getHeading(AngleUnit.RADIANS) + Math.PI /2));
        return pinpointPose;
    }
    public Pose updateAprilTagPose(){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            aprilTagDetected = true;
            aprilTagPose = new Pose(72 + convertMetersToInch(result.getBotpose().getPosition().y), 72 - convertMetersToInch(result.getBotpose().getPosition().x), normalizeAngleRad(result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI / 2));
        }
        else{
            aprilTagDetected = false;
        }
        return aprilTagPose;
    }

    public double normalizeAngleRad(double val){//keep within 0 - 2PI
        double angle = val % (2 * Math.PI);
        if(angle < 0){
            angle += Math.PI * 2;
        }
        return angle;
    }


    public Pose updateKalmanFilter(){
        if(aprilTagPose != null) {
            double fusedX = kalmanFilter.update(pinpointPose.getX(), aprilTagPose.getX(), aprilTagDetected);
            double fusedY = kalmanFilter.update(pinpointPose.getY(), aprilTagPose.getY(), aprilTagDetected);
            double fusedTheta = kalmanFilter.updateAngle(pinpointPose.getHeading(), aprilTagPose.getHeading(), aprilTagDetected);
            currentMergedPose = new Pose(fusedX, fusedY, normalizeAngleRad(fusedTheta));
        }
        else{
            double fusedX = kalmanFilter.update(pinpointPose.getX(), 0, aprilTagDetected);
            double fusedY = kalmanFilter.update(pinpointPose.getY(), 0, aprilTagDetected);
                double fusedTheta = kalmanFilter.updateAngle(pinpointPose.getHeading(), 0, aprilTagDetected);
            currentMergedPose = new Pose(fusedX, fusedY, fusedTheta);
        }
        return currentMergedPose;
    }

    @Override
    public Pose getPose() {
        return currentMergedPose;
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        this.startPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        this.currentMergedPose = setPose;
    }


    private double updateTotalYaw(){
        totalHeading += MathFunctions.getSmallestAngleDifference(currentMergedPose.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentMergedPose.getHeading());
        return totalHeading;
    }
    private Pose updateVelocityPose() {
        double deltaTime = timer.getDeltaTime();
        currentVelocity = new Pose((currentMergedPose.getX() - previousMergedPose.getX()) / deltaTime, (currentMergedPose.getY() - previousMergedPose.getY()) / deltaTime, (currentMergedPose.getHeading() - previousMergedPose.getHeading()) / deltaTime);
        return currentVelocity;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return pinpoint.getEncoderY();
    }

    @Override
    public double getLateralMultiplier() {
        return pinpoint.getEncoderX();
    }

    @Override
    public double getTurningMultiplier() {
        return pinpoint.getYawScalar();
    }

    @Override
    public void resetIMU() throws InterruptedException {

    }

    private double convertMetersToInch(double val){
        return val * 100 / 2.54;
    }

    @Override
    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }
}
