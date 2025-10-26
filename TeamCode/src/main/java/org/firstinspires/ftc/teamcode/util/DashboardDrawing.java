package org.firstinspires.ftc.teamcode.util;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Drawing class. It handles the drawing of stuff on FTC Dashboard, like the robot.
 *
 * @author Logan Nash
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/22/2024
 */
public class DashboardDrawing {
    public static final double ROBOT_RADIUS = 9;

    private static TelemetryPacket packet;
    public static List<Pose> poseHistory = new ArrayList<>();

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), "#3F51B5");
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), "#3F51B5");
        }
        drawPoseHistory(follower.getPoseTracker(), "#4CAF50");
        drawRobot(follower.getPose(), "#4CAF50");

        sendPacket();
    }

    /**
     * This adds instructions to the current packet to draw a robot at a specified Pose with a specified
     * color. If no packet exists, then a new one is created.
     *
     * @param pose the Pose to draw the robot at
     * @param color the color to draw the robot with
     */
    public static void drawRobot(Pose pose, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        DashboardDrawing.drawRobotOnCanvas(packet.fieldOverlay(), pose.copy());
    }

    /**
     * This adds instructions to the current packet to draw a Path with a specified color. If no
     * packet exists, then a new one is created.
     *
     * @param path the Path to draw
     * @param color the color to draw the Path with
     */
    public static void drawPath(Path path, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        DashboardDrawing.drawPath(packet.fieldOverlay(), path.getPanelsDrawingPoints());
    }

    /**
     * This adds instructions to the current packet to draw all the Paths in a PathChain with a
     * specified color. If no packet exists, then a new one is created.
     *
     * @param pathChain the PathChain to draw
     * @param color the color to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, String color) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), color);
        }
    }

    /**
     * This adds instructions to the current packet to draw the pose history of the robot. If no
     * packet exists, then a new one is created.
     *
     * @param poseTracker the DashboardPoseTracker to get the pose history from
     * @param color the color to draw the pose history with
     */
    public static void drawPoseHistory(PoseTracker poseTracker, String color) {
        if (packet == null) packet = new TelemetryPacket();
        if(poseHistory.size() < 2){
            return;
        }
        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().setStrokeWidth(1);
        for(int i = 0; i < poseHistory.size() - 1; i++) {
            Pose p1 = poseHistory.get(i);
            Pose p2 = poseHistory.get(i+1);
            packet.fieldOverlay().strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
        }
    }

    public static void addPoseToList(Pose pose){
        poseHistory.add(pose);
    }
    public static void clearPoseHistory(){
        poseHistory.clear();
    }

    /**
     * This tries to send the current packet to FTC Dashboard.
     *
     * @return returns if the operation was successful.
     */
    public static boolean sendPacket() {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = null;
            return true;
        }
        return false;
    }

    /**
     * This draws a robot on the Dashboard at a specified Point.
     *
     * @param c the Canvas on the Dashboard on which this will draw
     * @param t the Point to draw at
     */
    public static void drawRobotOnCanvas(Canvas c, Point t) {
        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        double theta = t.getThetaRadians();
        double x1 = t.getX() + Math.cos(theta) * ROBOT_RADIUS * 0.5;
        double y1 = t.getY() + Math.sin(theta) * ROBOT_RADIUS * 0.5;
        double x2 = t.getX() + Math.cos(theta) * ROBOT_RADIUS;
        double y2 = t.getY() + Math.sin(theta) * ROBOT_RADIUS;

        c.strokeLine(x1, y1, x2, y2);
    }


    /**
     * This draws a robot on the Dashboard at a specified Pose. This is more useful for drawing the
     * actual robot, since the Pose contains the direction the robot is facing as well as its position.
     *
     * @param c the Canvas on the Dashboard on which this will draw at
     * @param t the Pose to draw at
     */
    public static void drawRobotOnCanvas(Canvas c, Pose t) {
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);
        Vector v = t.getHeadingAsUnitVector();
        v.setMagnitude(ROBOT_RADIUS);
        double x1 = t.getX() + v.getXComponent() / 2, y1 = t.getY() + v.getYComponent() / 2;
        double x2 = t.getX() + v.getXComponent(), y2 = t.getY() + v.getYComponent();
        c.strokeLine(x1, y1, x2, y2);
    }

    /**
     * This draws a Path on the Dashboard from a specified Array of Points.
     *
     * @param c the Canvas on the Dashboard on which this will draw
     * @param points the Points to draw
     */
    public static void drawPath(Canvas c, double[][] points) {
        c.strokePolyline(points[0], points[1]);
    }
}