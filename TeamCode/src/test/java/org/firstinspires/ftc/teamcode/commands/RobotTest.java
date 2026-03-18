package org.firstinspires.ftc.teamcode.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

class RobotTest {
    @Test
    void testConfig() {
        Map<String, String> config = new HashMap<>();
        config.put("hello", "world");
        config.put("three", "musketeers");
        Robot.config = config;
        assertEquals(Robot.config.get("hello"), "world");
        assertEquals(Robot.config.get("three"), "musketeers");
    }
}