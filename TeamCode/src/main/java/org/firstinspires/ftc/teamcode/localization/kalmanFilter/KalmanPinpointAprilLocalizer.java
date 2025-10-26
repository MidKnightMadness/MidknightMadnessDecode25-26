package org.firstinspires.ftc.teamcode.localization.kalmanFilter;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Config
@Configurable
public class KalmanPinpointAprilLocalizer implements Localizer {
    KalmanPinpointAprilConstants constants;
    private Pose startPose;
    private Pose currentMergedPose = new Pose();
    private Pose aprilTagPose;
    private Pose pinpointPose;
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
    Timer timer;
    double previousTime;

    double leftAprilTagLeftThreshold = 0;//inches from the center of the field that a close to 0 degrees(facing toward motif) will swap to one side


    public KalmanPinpointAprilLocalizer(HardwareMap hardwareMap, KalmanPinpointAprilConstants lConstants){
        this(hardwareMap, lConstants, new Pose());
    }

    public KalmanPinpointAprilLocalizer(HardwareMap hardwareMap, KalmanPinpointAprilConstants lConstants, Pose startPose) {
        this.startPose = startPose;
        this.constants = lConstants;
        this.hardwareMap = hardwareMap;

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
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startPose.getX(), startPose.getY(),AngleUnit.RADIANS, AngleUnit.normalizeRadians(startPose.getHeading()-Math.PI/2)));
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

    double wrapAngleRad(double value){
        while(value <= - Math.PI){
            value += 2 * Math.PI;
        }
        while(value >=  Math.PI){
            value -=  2 * Math.PI;
        }
        return value;
    }


    private Pose convertMetersToInch(Pose pose){
        double x = pose.getX() * 100 / 2.54;
        double y = pose.getY() * 100 / 2.54;
        return new Pose(x, y, pose.getHeading());

    }


    @Override
    public Pose getPose() {
        return currentMergedPose;
    }

    public Pose updateKalmanFilter(){
        if(aprilTagPose != null) {
            double fusedX = kalmanFilter.update(pinpointPose.getX(), aprilTagPose.getX(), aprilTagDetected);
            double fusedY = kalmanFilter.update(pinpointPose.getY(), aprilTagPose.getY(), aprilTagDetected);
            double fusedTheta = kalmanFilter.updateAngle(pinpointPose.getHeading(), aprilTagPose.getHeading(), aprilTagDetected);
            currentMergedPose = new Pose(fusedX, fusedY, fusedTheta);
        }
        else{
            double fusedX = kalmanFilter.update(pinpointPose.getX(), 0, aprilTagDetected);
            double fusedY = kalmanFilter.update(pinpointPose.getY(), 0, aprilTagDetected);
            double fusedTheta = kalmanFilter.updateAngle(pinpointPose.getHeading(), 0, aprilTagDetected);
            currentMergedPose = new Pose(fusedX, fusedY, fusedTheta);
        }
        return currentMergedPose;
    }
    public Pose updateAprilTagPose(){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            aprilTagDetected = true;
            aprilTagPose = convertMetersToInch(new Pose(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y, result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));
        }
        else{
            aprilTagDetected = false;
        }
        if(aprilTagPose != null) {
            aprilTagPose = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, aprilTagPose.getX(), aprilTagPose.getY(), AngleUnit.RADIANS, aprilTagPose.getHeading()), PedroCoordinates.INSTANCE);
        }
        return aprilTagPose;
    }

    public Pose updatePinpointPose(){
        pinpointPose = new Pose(2 * startPose.getX() - pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getPosX(DistanceUnit.INCH), (pinpoint.getHeading(AngleUnit.RADIANS) + Math.PI/2 + 2 * Math.PI) % Math.PI);
        return pinpointPose;
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

    @Override
    public void update() {

        if(!limelight.isRunning()){//if the limelight is not currently running a pipeline, we start running it
            limelight.start();
        }
        if(limelight.isRunning()) {
//            if ((pinpoint.getHeading(AngleUnit.RADIANS) > 0) || (Math.abs(pinpoint.getHeading(AngleUnit.DEGREES)) < 5 && currentMergedPose.getY() <= leftAprilTagLeftThreshold)) {
            if ((pinpoint.getHeading(AngleUnit.RADIANS) > 0)){
                limelight.pipelineSwitch(constants.leftPipelineNum);
            } else {
                limelight.pipelineSwitch(constants.rightPipelineNum);
            }
        }

        updatePinpointPose();
        updateAprilTagPose();
        updateKalmanFilter();
        updateVelocityPose();
        updateTotalYaw();

        previousHeading = currentMergedPose.getHeading();
        previousMergedPose = currentMergedPose;

    }

    private double updateTotalYaw(){
        totalHeading += MathFunctions.getSmallestAngleDifference(currentMergedPose.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentMergedPose.getHeading());
        return totalHeading;
    }
    private Pose updateVelocityPose() {
        double deltaTime = timer.getDeltaTime(TimeUnit.SECONDS);
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

    @Override
    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }
}
