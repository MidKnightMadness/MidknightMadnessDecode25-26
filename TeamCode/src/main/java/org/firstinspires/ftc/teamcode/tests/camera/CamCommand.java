package org.firstinspires.ftc.teamcode.tests.camera;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.List;

@Config
@Configurable
public class CamCommand extends CommandBase {

    public CamCommand(Limelight3A limelight, Follower follower, ShootSide shootSide){
        this.limelight = limelight;
        this.follower = follower;
        ballList = new ArrayList<>();
        globalPoseList = new ArrayList<>();
        finalGlobalPoseList = new ArrayList<>();
        finalBallList = new ArrayList<>();
        this.shootSide = shootSide;
    }
    ArrayList<Pose> ballList;
    public ArrayList<Pose> finalBallList;
    Follower follower;
    Pose ballPose;
    Pose minBall;
    Limelight3A limelight;
    LLResult result;
    List<LLResultTypes.DetectorResult> detections;

    double minX;
    double minY;
    double minD;
    private static final double horizontalFOV = 54.5;
    private static final double verticalFOV = 42;
    private static final double camWidth = 1280;
    private static final double camHeight = 960;
    private static final double PPI = 96;
    private static double hOffset = -3.5;
    private static double vOffset = 15;
    public static double intakeFromCenterY = 0;
    public static double intakeFromCenterX = 8.5;
    ShootSide shootSide;
    public static double minBallDetect = 2;
    Timer timer;
//    long minWaitTime = 500;
    private final double[][] H = {
            {5.828680, 1.841763, -3515.724491},
            {0.052028, 13.011093, -4470.243506},
            {0.000090, 0.003892, 1.000000}
    };

    ArrayList<Pose> globalPoseList;
    ArrayList<Pose> finalGlobalPoseList;

    private static double distanceBot(Pose pose) {
        return Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY());
    }



    private Pose processHomography(double x_degrees, double y_degrees) {//this takes degrees and sets coordinateX and Y to the homographied thing
        double x = (x_degrees / horizontalFOV) * camWidth + (camWidth / 2.0);
        double y = (y_degrees / verticalFOV) * camHeight + (camHeight / 2.0);

        double X_prime = H[0][0] * x + H[0][1] * y + H[0][2];
        double Y_prime = H[1][0] * x + H[1][1] * y + H[1][2];
        double W = H[2][0] * x + H[2][1] * y + H[2][2];

        double x_robot = X_prime / W / PPI + hOffset;
        double y_robot = Y_prime / W / PPI + vOffset;

        return new Pose(x_robot, y_robot);

    }
    private Pose coordRobotToField(Follower follower, Pose poseBall){
        follower.update();
        Pose botPose = follower.getPose();
//        double dist = distanceBot(poseBall);

//        double xb = dist * Math.cos(poseBall.getHeading());
//        double yb = dist * Math.sin(poseBall.getHeading());
        double xb = poseBall.getY();
        double yb = -poseBall.getX();
        if(xb < 4){
            xb += 5;
        } if(yb < 4){
            yb += 5;
        }

        double x = botPose.getX() + Math.cos(botPose.getHeading()) * xb - Math.sin(botPose.getHeading()) * yb;//from bot but in field
        double y = botPose.getY() + Math.sin(botPose.getHeading()) * xb + Math.cos(botPose.getHeading()) * yb;

        if(shootSide == ShootSide.RIGHT) {
            x -= (intakeFromCenterX * Math.cos(botPose.getHeading()) - Math.sin(botPose.getHeading()) * intakeFromCenterY);
            y -= (intakeFromCenterX * Math.sin(botPose.getHeading()) + Math.cos(botPose.getHeading()) * intakeFromCenterY);
        } else{
            x += (intakeFromCenterX * Math.cos(botPose.getHeading()) - Math.sin(botPose.getHeading()) * intakeFromCenterY);
            y += (intakeFromCenterX * Math.sin(botPose.getHeading()) + Math.cos(botPose.getHeading()) * intakeFromCenterY);

        }

        return new Pose(x, y, botPose.getHeading());
    }


    @Override
    public void initialize() {
        limelight.setPollRateHz(400); //default 100
        limelight.start();
        limelight.pipelineSwitch(3);
        minX = Double.MAX_VALUE;
        minY = Double.MAX_VALUE;
        minD = Double.MAX_VALUE;
        minBall = new Pose(Double.MAX_VALUE, Double.MAX_VALUE);
        timer = new Timer();
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();
        if(result != null) {
            detections = result.getDetectorResults();
        }

        if (result != null && detections != null && !detections.isEmpty()) {
            for (LLResultTypes.DetectorResult detection : detections) {
//                String className = detection.getClassName(); // What was detected
                double x = detection.getTargetXDegrees(); // Where it is (left-right)
                double y = detection.getTargetYDegrees(); // Where it is (up-down)
                ballPose = processHomography(x, y);

                ballList.add(ballPose);
                globalPoseList.add(coordRobotToField(follower, ballPose));

//                if (distanceBot(ballPose) < minD) {
//                    minBall = new Pose(ballPose.getX(), ballPose.getY(), 0);
//                    minBall = coordRobotToField(follower, ballPose);
//                    minD = distanceBot(ballPose);
//                }
            }
            minD = Double.MAX_VALUE;

            finalBallList = new ArrayList<>(ballList);
            finalGlobalPoseList = new ArrayList<>(globalPoseList);
            ballList.clear();
            globalPoseList.clear();

        }
    }
    public ArrayList<Pose> getFinalGlobalPoseList(){
        return finalGlobalPoseList;
    }

    public ArrayList<Pose> getFinalBallList(){
        return finalBallList;
    }
    boolean start = false;
    double startTime = 0;
    @Override
    public boolean isFinished(){//manually do it in auto
        if(finalGlobalPoseList != null && finalGlobalPoseList.size() >= minBallDetect){
            return true;
        } return false;
//            start = true;
//            startTime = timer.getTime();
//        }
//        return (start && timer.getDeltaTime() - startTime >= minWaitTime);
    }

    public Pose getMinBallPose(){
        return minBall;
    }

    public PathChain getList(Follower follower, double targetX){
        if(finalGlobalPoseList == null || finalGlobalPoseList.size() < minBallDetect){
            return null;
        }

        ArrayList<Pose> balls = new ArrayList<>(finalGlobalPoseList);


        Pose correctTargetPose = new Pose(targetX, balls.get(0).getY(), balls.get(0).getHeading());
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), correctTargetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), correctTargetPose.getHeading())
                .build();

        return path;
    }
    public Pose getHomographyPose(){
        if(finalBallList.get(0) != null) {
            return processHomography(finalBallList.get(0).getX(), finalBallList.get(0).getY());
        }
        else{
            return new Pose(-1, -1);
        }
    }
}
