package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.jupiter.api.Test;

class AngleTest {

    @Test
    void basic() {
        Angle a1 = Angle.fromDegrees(180);
        assertEquals(180, a1.toDegrees(), 0.001);
        assertEquals(Math.PI, a1.toRadians(), 0.001);
        assertEquals(180, a1.to(AngleUnit.DEGREES), 0.001);
        assertEquals(Math.PI, a1.to(AngleUnit.RADIANS), 0.001);

        Angle a2 = Angle.fromRadians(-Math.PI);
        assertEquals(-180, a2.toDegrees(), 0.001);
        assertEquals(-Math.PI, a2.toRadians(), 0.001);
        assertEquals(-180, a2.to(AngleUnit.DEGREES), 0.001);
        assertEquals(-Math.PI, a2.to(AngleUnit.RADIANS), 0.001);
    }

    @Test
    void wrapDegrees() {
        Angle a1 = Angle.fromDegrees(500);
        assertEquals(140, a1.wrap().toDegrees(), 0.001);

        Angle a2 = Angle.fromDegrees(600);
        assertEquals(-120, a2.wrap().toDegrees(), 0.001);

        Angle a3 = Angle.fromDegrees(135);
        assertEquals(135, a3.wrap().toDegrees(), 0.001);

        Angle a4 = Angle.fromDegrees(-500);
        assertEquals(-140, a4.wrap().toDegrees(), 0.001);

        Angle a5 = Angle.fromDegrees(-600);
        assertEquals(120, a5.wrap().toDegrees(), 0.001);

        Angle a6 = Angle.fromDegrees(-135);
        assertEquals(-135, a6.wrap().toDegrees(), 0.001);
    }

    @Test
    void wrapRadians() {
        Angle a1 = Angle.fromRadians(2.5 * Math.PI);
        assertEquals(0.5 * Math.PI, a1.wrap().toRadians(), 0.001);

        Angle a2 = Angle.fromRadians(3.5 * Math.PI);
        assertEquals(-0.5 * Math.PI, a2.wrap().toRadians(), 0.001);

        Angle a3 = Angle.fromRadians(0.5 * Math.PI);
        assertEquals(0.5 * Math.PI, a3.wrap().toRadians(), 0.001);

        Angle a4 = Angle.fromRadians(-2.5 * Math.PI);
        assertEquals(-0.5 * Math.PI, a4.wrap().toRadians(), 0.001);

        Angle a5 = Angle.fromRadians(-3.5 * Math.PI);
        assertEquals(0.5 * Math.PI, a5.wrap().toRadians(), 0.001);

        Angle a6 = Angle.fromRadians(-0.5 * Math.PI);
        assertEquals(-0.5 * Math.PI, a6.wrap().toRadians(), 0.001);
    }

    @Test
    void wrapUnsignedDegrees() {
        Angle a1 = Angle.fromDegrees(500);
        assertEquals(140, a1.wrapUnsigned().toDegrees(), 0.001);

        Angle a2 = Angle.fromDegrees(600);
        assertEquals(240, a2.wrapUnsigned().toDegrees(), 0.001);

        Angle a3 = Angle.fromDegrees(135);
        assertEquals(135, a3.wrapUnsigned().toDegrees(), 0.001);

        Angle a4 = Angle.fromDegrees(-500);
        assertEquals(220, a4.wrapUnsigned().toDegrees(), 0.001);

        Angle a5 = Angle.fromDegrees(-600);
        assertEquals(120, a5.wrapUnsigned().toDegrees(), 0.001);

        Angle a6 = Angle.fromDegrees(-135);
        assertEquals(225, a6.wrapUnsigned().toDegrees(), 0.001);
    }

    @Test
    void wrapUnsignedRadians() {
        Angle a1 = Angle.fromRadians(2.5 * Math.PI);
        assertEquals(0.5 * Math.PI, a1.wrapUnsigned().toRadians(), 0.001);

        Angle a2 = Angle.fromRadians(3.5 * Math.PI);
        assertEquals(1.5 * Math.PI, a2.wrapUnsigned().toRadians(), 0.001);

        Angle a3 = Angle.fromRadians(0.5 * Math.PI);
        assertEquals(0.5 * Math.PI, a3.wrapUnsigned().toRadians(), 0.001);

        Angle a4 = Angle.fromRadians(-2.5 * Math.PI);
        assertEquals(1.5 * Math.PI, a4.wrapUnsigned().toRadians(), 0.001);

        Angle a5 = Angle.fromRadians(-3.5 * Math.PI);
        assertEquals(0.5 * Math.PI, a5.wrapUnsigned().toRadians(), 0.001);

        Angle a6 = Angle.fromRadians(-0.5 * Math.PI);
        assertEquals(1.5 * Math.PI, a6.wrapUnsigned().toRadians(), 0.001);
    }

    @Test
    void add() {
        Angle a1 = Angle.fromRadians(1.5 * Math.PI);
        Angle a2 = Angle.fromDegrees(180);
        assertEquals(450, a1.add(a2).toDegrees(), 0.001);
    }

    @Test
    void sub() {
        Angle a1 = Angle.fromRadians(-1.5 * Math.PI);
        Angle a2 = Angle.fromDegrees(180);
        assertEquals(-450, a1.sub(a2).toDegrees(), 0.001);
    }

    @Test
    void delta() {
        Angle a = Angle.fromDegrees(123);
        Angle b = Angle.fromDegrees(456);

        assertEquals(
                a.delta(b).toDegrees(),
                -b.delta(a).toDegrees(),
                0.001
        );
    }

    @Test
    void distance() {
        Angle a = Angle.fromDegrees(123);
        Angle b = Angle.fromDegrees(456);

        assertEquals(
                a.distance(b).toDegrees(),
                b.distance(a).toDegrees(),
                0.001
        );

        assertEquals(20, Angle.fromDegrees(-10).distance(Angle.fromDegrees(10)).toDegrees(), 0.001);
        assertEquals(40, Angle.fromDegrees(-170).distance(Angle.fromDegrees(150)).toDegrees(), 0.001);
        assertEquals(0, Angle.fromDegrees(0).distance(Angle.fromDegrees(360)).toDegrees(), 0.001);
        assertEquals(0, Angle.fromDegrees(720).distance(Angle.fromDegrees(0)).toDegrees(), 0.001);
        assertEquals(20, Angle.fromDegrees(720 + 10).distance(Angle.fromDegrees(-10)).toDegrees(), 0.001);
    }

    @Test
    void abs() {
        assertEquals(30, Angle.fromDegrees(-30).abs().toDegrees(), 0.001);
        assertEquals(45, Angle.fromDegrees(45).abs().toDegrees(), 0.001);
    }

    @Test
    void neg() {
        assertEquals(-30, Angle.fromDegrees(30).neg().toDegrees(), 0.001);
        assertEquals(30, Angle.fromDegrees(-30).neg().toDegrees(), 0.001);
    }

    @Test
    void sign() {
        assertEquals(1, Angle.fromDegrees(10).sign());
        assertEquals(-1, Angle.fromDegrees(-10).sign());
        assertEquals(0, Angle.fromDegrees(0).sign());
    }

    @Test
    void less() {
        assertTrue(Angle.fromDegrees(10).less(Angle.fromDegrees(20)));
        assertFalse(Angle.fromDegrees(20).less(Angle.fromDegrees(10)));
    }

    @Test
    void greater() {
        assertTrue(Angle.fromDegrees(20).greater(Angle.fromDegrees(10)));
        assertFalse(Angle.fromDegrees(10).greater(Angle.fromDegrees(20)));
    }

    @Test
    void le() {
        assertTrue(Angle.fromDegrees(10).le(Angle.fromDegrees(10)));
        assertTrue(Angle.fromDegrees(10).le(Angle.fromDegrees(20)));
        assertFalse(Angle.fromDegrees(20).le(Angle.fromDegrees(10)));
    }

    @Test
    void ge() {
        assertTrue(Angle.fromDegrees(10).ge(Angle.fromDegrees(10)));
        assertTrue(Angle.fromDegrees(20).ge(Angle.fromDegrees(10)));
        assertFalse(Angle.fromDegrees(10).ge(Angle.fromDegrees(20)));
    }

    @Test
    void atAngle() {
        assertTrue(Angle.fromDegrees(90).atAngle(Angle.fromDegrees(90), Angle.fromDegrees(0)));
        assertTrue(Angle.fromDegrees(91).atAngle(Angle.fromDegrees(90), Angle.fromDegrees(2)));
        assertFalse(Angle.fromDegrees(95).atAngle(Angle.fromDegrees(90), Angle.fromDegrees(2)));
    }
}