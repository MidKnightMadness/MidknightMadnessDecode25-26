package org.firstinspires.ftc.teamcode.commands;

import android.content.Context;
import android.content.res.AssetManager;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

/**
 * This is the Robot class. This will make your command-based robot code a lot smoother
 * and easier to understand.
 */
public class Robot {

    public static boolean isDisabled = false;
    public static final Map<String, String> config = new HashMap<>();

    /**
     * Loads .env style config directly from a {@link InputStream}
     *
     * @return The config map
     */
    public static Map<String, String> loadConfig(InputStream is) {
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(is))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split("=", 2);
                if (parts.length == 2) {
                    config.put(parts[0].trim(), parts[1].trim());
                }
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return config;
    }

    /**
     * Loads .env style config from <code>src/main/assets/fileName</code>
     *
     * @param context Android context (use <code>hardwareMap.appContext</code>
     * @param fileName The name of the file
     * @return The config map
     */
    public static Map<String, String> loadConfig(Context context, String fileName) {
        config.clear();
        AssetManager am = context.getAssets();

        try (InputStream is = am.open(fileName)) {
            return loadConfig(is);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static String getConfigVar(String key) {
        return config.get(key);
    }

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules {@link Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link Subsystem} objects to the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    public static void disable() {
        isDisabled = true;
    }

    public static void enable() {
        isDisabled = false;
    }

    /**
     * Method to automatically set all hubs to bulk read, greatly reducing loop times.
     * @param hwMap hardwareMap to access hub objects
     * @param cachingMode the mode in which the hubs operate during bulk reading/caching.
     *                    MANUAL mode is highly recommended and comes to the user with no
     *                    extra work and doesn't read any hardware more than once per loop,
     *                    while AUTO enables bulk reads but will conduct a bulk read any time
     *                    a specific hardware is read the second time, even in a loop,
     *                    potentially leading to worse loop times.
     */
    public void setBulkReading(HardwareMap hwMap, LynxModule.BulkCachingMode cachingMode) {
        CommandScheduler.getInstance().setBulkReading(hwMap, cachingMode);
    }

}