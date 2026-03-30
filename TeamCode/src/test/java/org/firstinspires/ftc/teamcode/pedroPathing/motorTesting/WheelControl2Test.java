package org.firstinspires.ftc.teamcode.pedroPathing.motorTesting;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.newpid.PIDController;
import org.junit.jupiter.api.Test;


public class WheelControl2Test {

    PIDController driveController = new PIDController(0.03, 0, 0.001);
    PIDController headingController = new PIDController(0.5, 0, 0.01);

    @Test
    void pid() {
        Pose robotPose = new Pose(8, 7, Math.toRadians(100));
        Pose target = new Pose(30, 7, 0);
        double drivePower = driveController.calculate(robotPose.distanceFrom(target));
        double strafeAngle = target.minus(robotPose).getAsVector().getTheta();
        double headingPower = headingController.calculate(
                MathFunctions.normalizeAngleSigned(
                        robotPose.getHeading() - target.getHeading()
                )
        );

        double theta = strafeAngle - robotPose.getHeading();
        double forward = drivePower*Math.cos(theta);
        double right = -drivePower*Math.sin(theta);

        assertEquals(0.66, drivePower, 0.001);
        assertEquals(0, strafeAngle, 0.001);
        assertEquals(0.872665, headingPower, 0.001);
        assertEquals(-0.11461, forward, 0.001);
        assertEquals(0.649973, right, 0.001);
    }
}