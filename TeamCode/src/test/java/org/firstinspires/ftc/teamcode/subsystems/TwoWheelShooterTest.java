package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter.AimCalculator;
import org.junit.jupiter.api.Test;

import static org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter.leftShootPose;
import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

class TwoWheelShooterTest {
    @Test
    void pedroVectorTest() {
        Vector vector1 = new Vector(3, 0);
        System.out.println(vector1);
        Vector vector2 = vector1.normalize();
        System.out.println(vector2);
        Vector vector3 = vector2.copy();
        vector3.rotateVector(-Math.PI / 2);
        System.out.println(vector3);
        System.out.println(vector1.dot(vector3));
    }

    @Test
    void aimCalculatorTest() {
        AimCalculator aimCalculator = new AimCalculator();
        double[] aimData = aimCalculator.targetPowersHeading(
                new Pose(0, 0, 0),
                new Vector(1, 0),
                leftShootPose
        );
        double[] aimData2 = aimCalculator.targetPowersHeading(
                new Pose(0, 0, 0),
                new Vector(1, 0),
                leftShootPose
        );
        System.out.println(aimData[0] + ", " + aimData[1] + ", " + aimData[2]);
        assertEquals(aimData[0], aimData2[0]);
        assertEquals(aimData[1], aimData2[1]);
        assertEquals(aimData[2], aimData2[2]);
    }
}