package org.firstinspires.ftc.teamcode.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Localization.KalmanFilter;
import org.firstinspires.ftc.teamcode.Localization.KalmanPinpointAprilConstants;
@Config
@Configurable
public class KalmanPinpointAprilLocalizer implements Localizer {
    public boolean MOTIF_DETECTION = false;
    KalmanPinpointAprilConstants constants;
    private Pose startPose;
    private Pose currentMergedPose;
    private Pose aprilTagPose;
    private Pose pinpointPose;
    HardwareMap hardwareMap;
    IMU imu;
    GoBildaPinpointDriver pinpoint;
    KalmanFilter kalmanFilter;
    Limelight3A limelight;
    boolean motifDetecting = false;
    boolean aprilTagDetected = false;

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
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startPose.getX(), startPose.getY(), AngleUnit.RADIANS, startPose.getHeading()));
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(constants.xDir, constants.yDir);
        pinpoint.setOffsets(constants.xOffset, constants.yOffset, constants.distUnit);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }


    private Pose convertMetersToInch(Pose pose){
        double x = pose.getX() * 100 / 2.54;
        double y = pose.getY() * 100 / 2.54;
        return new Pose(x, y, pose.getHeading());

    }


    @Override
    public Pose getPose() {
        if(!limelight.isRunning()){//if the limelight is not currently running a pipeline, we start running it
            limelight.start();
        }
        if(limelight.isRunning()) {
            if ((pinpoint.getHeading(AngleUnit.RADIANS) > 0) || (Math.abs(pinpoint.getHeading(AngleUnit.DEGREES)) < 5 && currentMergedPose.getY() <= leftAprilTagLeftThreshold)) {
                limelight.pipelineSwitch(constants.leftPipelineNum);
            } else {
                limelight.pipelineSwitch(constants.rightPipelineNum);
            }
        }

        updatePinpointPose();
        updateAprilTagPose();
        updateKalmanFilter();
        return currentMergedPose;
    }

    public Pose updateKalmanFilter(){
        if(!aprilTagDetected) {
            double fusedX = kalmanFilter.update(pinpointPose.getX(), aprilTagPose.getX(), aprilTagDetected);
            double fusedY = kalmanFilter.update(pinpointPose.getY(), aprilTagPose.getY(), aprilTagDetected);
            double fusedTheta = kalmanFilter.updateAngle(pinpointPose.getHeading(), aprilTagPose.getHeading(), aprilTagDetected);
            currentMergedPose = new Pose(fusedX, fusedY, fusedTheta);
        }
        return currentMergedPose;
    }
    public Pose updateAprilTagPose(){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            aprilTagDetected = true;
            aprilTagPose = new Pose(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y, result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
            aprilTagPose = convertMetersToInch(aprilTagPose);
        }
        return aprilTagPose;
    }
    public Pose updatePinpointPose(){
        pinpointPose = new Pose(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.RADIANS));
        return pinpointPose;
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {

    }

    @Override
    public void setPose(Pose setPose) {

    }

    @Override
    public void update() {

    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        imu.resetYaw();
    }

    @Override
    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public boolean isNAN() {
        return false;
    }
}
