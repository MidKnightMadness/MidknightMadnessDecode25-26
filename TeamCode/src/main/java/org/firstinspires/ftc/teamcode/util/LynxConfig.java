package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public abstract class LynxConfig {
    protected final Map<String, Class<? extends HardwareDevice>> classMap = new HashMap<>();
    protected final Map<String, String> portMap = new HashMap<>();

    protected <T extends HardwareDevice> void addDevice(String name, Class<T> type, String port) {
        classMap.put(name, type);
        portMap.put(name, port);
    }

    protected <T extends HardwareDevice> void addDevice(String name, Class<T> type) {
        classMap.put(name, type);
    }

    public Class<? extends HardwareDevice> getClass(String name) {
        return classMap.get(name);
    }

    public String getPort(String name) {
        return portMap.get(name);
    }

    public Set<String> getAllNames() {
        return Collections.unmodifiableSet(portMap.keySet());
    }

    @SuppressWarnings("unchecked")
    public <T extends HardwareDevice> T get(HardwareMap hardwareMap, String name) {
        Class<? extends HardwareDevice> clazz = getClass(name);
        if (clazz == null) {
            throw new IllegalArgumentException("Device not registered: " + name);
        }
        return (T) hardwareMap.get(clazz, name);
    }
}
