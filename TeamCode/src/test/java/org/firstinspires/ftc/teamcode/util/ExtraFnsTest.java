package org.firstinspires.ftc.teamcode.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

class ExtraFnsTest {

    @Test
    void firstSupplier() {
        AtomicBoolean inner = new AtomicBoolean(false);
        BooleanSupplier s = ExtraFns.firstSupplier(inner::get);
        assertFalse(s.getAsBoolean());
        assertFalse(s.getAsBoolean());
        assertFalse(s.getAsBoolean());
        inner.set(true);
        assertTrue(s.getAsBoolean());
        assertFalse(s.getAsBoolean());
        inner.set(false);
        assertFalse(s.getAsBoolean());
        inner.set(true);
        assertFalse(s.getAsBoolean());
        assertFalse(s.getAsBoolean());
    }
}