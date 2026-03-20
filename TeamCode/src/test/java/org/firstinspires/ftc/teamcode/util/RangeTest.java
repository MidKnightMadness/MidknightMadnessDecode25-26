package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RangeTest {
    @Test
    void getValueRatio() {
        Range range = new Range(-10, 10);
        assertEquals(range.getValueRatio(4), 0.7);
    }

    @Test
    void getValueFromRatio() {
        Range range = new Range(-10, 10);
        assertEquals(range.getValueFromRatio(0.7), 4);
    }

    @Test
    void convert() {
        Range range = new Range(-10, 10);
        Range newRange = new Range(-1, 1);
        assertEquals(range.convert(4, newRange), 0.4, 0.001);
    }

    @Test
    void clip() {
        Range range = new Range(-10, 10);
        assertEquals(range.clip(-12.1), -10);
        assertEquals(range.clip(13.9), 10);
        assertEquals(range.clip(7.4), 7.4);
    }

    @Test
    void range() {
        Range range = new Range(-10, 10);
        assertEquals(range.range(), 20);
    }

    @Test
    void contains() {
        Range range = new Range(-10, 10);
        assertTrue(range.contains(3));
        assertFalse(range.contains(-12));
        assertFalse(range.contains(12));
    }
}