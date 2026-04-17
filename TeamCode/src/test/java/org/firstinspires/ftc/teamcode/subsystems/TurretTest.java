package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Range;
import org.junit.jupiter.api.Test;

class TurretTest {
    static Range angleRange = new Range(Math.toRadians(-200), Math.toRadians(200));

    Angle optimizeAngle(Angle currentAngle, Angle targetAngle) {
        Angle bestCandidate = new Angle();
        Angle targetAngleWrapped = targetAngle.wrap();
        Angle minGap = Angle.fromRadians(Double.MAX_VALUE);
        Angle[] candidates = {
                targetAngleWrapped,
                targetAngleWrapped.sub(Angle.fromRadians(2 * Math.PI)),
                targetAngleWrapped.add(Angle.fromRadians(2 * Math.PI))
        };

        for (Angle candidate : candidates) {
            if (!angleRange.contains(candidate.toRadians())) continue;
            Angle gap = currentAngle.sub(candidate).abs();
            if (gap.less(minGap)) {
                minGap = gap;
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }

    @Test
    void optimizeAngle() {
        assertEquals(-200, optimizeAngle(Angle.fromDegrees(-190), Angle.fromDegrees(-200)).toDegrees(), 0.001);
        assertEquals(-190, optimizeAngle(Angle.fromDegrees(-170), Angle.fromDegrees(170)).toDegrees(), 0.001);
        assertEquals(190, optimizeAngle(Angle.fromDegrees(170), Angle.fromDegrees(-170)).toDegrees(), 0.001);
        assertEquals(-40, optimizeAngle(Angle.fromDegrees(-190), Angle.fromDegrees(-400)).toDegrees(), 0.001);
        assertEquals(40, optimizeAngle(Angle.fromDegrees(50), Angle.fromDegrees(400)).toDegrees(), 0.001);
        assertEquals(50, optimizeAngle(Angle.fromDegrees(50), Angle.fromDegrees(50)).toDegrees(), 0.001);
    }
}