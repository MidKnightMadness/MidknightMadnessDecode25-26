package org.firstinspires.ftc.teamcode.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import java.io.ByteArrayInputStream;
import java.io.InputStream;

class RobotTest {
    @Test
    void testConfig() {
        String config = "HELLO=world\nTHREE=musketeers";
        InputStream inputStream = new ByteArrayInputStream(config.getBytes());
        Robot.loadConfig(inputStream);
        assertEquals(Robot.getConfigVar("HELLO"), "world");
        assertEquals(Robot.getConfigVar("THREE"), "musketeers");
    }
}