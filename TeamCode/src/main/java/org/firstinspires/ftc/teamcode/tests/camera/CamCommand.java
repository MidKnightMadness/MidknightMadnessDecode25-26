package org.firstinspires.ftc.teamcode.camera;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.ArrayList;
import java.util.List;

public class CamCommand extends CommandBase {

    public CamCommand(Limelight3A limelight, Follower follower){
        this.limelight = limelight;
        this.follower = follower;
        ballList = new ArrayList<>();
        finalBallList = new ArrayList<>();
    }
    ArrayList<Pose> ballList;
    public static ArrayList<Pose> finalBallList;
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
    private final double[][] H = {
            {5.828680, 1.841763, -3515.724491},
            {0.052028, 13.011093, -4470.243506},
            {0.000090, 0.003892, 1.000000}
    };


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
        Pose botPose = follower.getPose();
        double hBallF = botPose.getHeading()-90+poseBall.getHeading();
        double distance = distanceBot(poseBall);

        double x = Math.cos(hBallF) * distance;//from bot but in field
        double y = Math.sin(hBallF) * distance;

        x += botPose.getX();
        y += botPose.getY();

        return new Pose(x, y, hBallF);
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
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();
        if(result != null) {
            detections = result.getDetectorResults();
        }

        if (!detections.isEmpty() && result != null) {
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName(); // What was detected
                double x = detection.getTargetXDegrees(); // Where it is (left-right)
                double y = detection.getTargetYDegrees(); // Where it is (up-down)
                ballPose = processHomography(x, y);
                if (distanceBot(ballPose) < minD) {
                    minBall = new Pose(ballPose.getX(), ballPose.getY(), 0);
                    minBall = coordRobotToField(follower, ballPose);
                    minD = distanceBot(ballPose);
                }
                ballPose = coordRobotToField(follower, ballPose);
                ballList.add(ballPose);
            }
            minD = Double.MAX_VALUE;

            finalBallList = new ArrayList<>(ballList);
            ballList.clear();

        }
    }
    @Override
    public boolean isFinished(){//manually do it in auto

        return false;
    }
    public Pose getMinBallPose(){
        return minBall;
    }

    public static PathChain getList(Follower follower){
        ArrayList<Path> lineList = new ArrayList<>();
        for(int i = 0; i < finalBallList.size()-1; i++){
            if(i == 0){
                lineList.add(new Path(new BezierLine(follower.getPose(), finalBallList.get(1))));
            }
            else{
                lineList.add(new Path(new BezierLine(finalBallList.get(i), finalBallList.get(i+1))));
            }
        }
        Path[] paths = lineList.toArray(new Path[0]);
        return new PathChain(paths);
    }
}
