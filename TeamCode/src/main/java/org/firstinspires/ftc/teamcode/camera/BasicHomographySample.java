package org.firstinspires.ftc.teamcode.camera;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.Point;

/**
 * BasicHomographySample demonstrates using a homography transformation
 * with a Limelight camera to convert camera coordinates to robot-relative coordinates.
 */ //nice chatgpt allen (i know where u live)
@TeleOp(name = "BasicHomographySample", group = "Linear OpMode")
public class BasicHomographySample extends LinearOpMode {

    // Offset constants in inches
    public static double HORIZONTAL_OFFSET = 0;
    public static double VERTICAL_OFFSET = 24;

    // Camera resolution constants
    public static double CAMERA_WIDTH = 640.0;
    public static double CAMERA_HEIGHT = 480.0;

    // Camera field of view constants in degrees
    public static double HORIZONTAL_FOV = 54.5;
    public static double VERTICAL_FOV = 42.0;

    // Homography transformation matrix
    private final double[][] H = {
            {4.158013, 5.076947, -1146.668632},
            {-1.674058, 8.920382, -450.058258},
            {-0.000171, 0.003455, 1.000000}
    };

    // Pixels Per Inch on calibration image
    private final double PPI = 96.0;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        initializeLimelight();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                ArrayList<Item> results = processLimelightResults();
                displayResults(results);
            }
        }
    }

    /**
     * Configures and starts the Limelight camera.
     */
    private void initializeLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Processes the latest results from the Limelight camera.
     *
     * @return ArrayList of detected items with robot-relative coordinates
     */
    private ArrayList<Item> processLimelightResults() {
        LLResult results = limelight.getLatestResult();
        ArrayList<Item> items = new ArrayList<>();

        if (results != null && results.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = results.getDetectorResults();

            for (LLResultTypes.DetectorResult result : detectorResults) {
                Point robotCoordinates = calculateRobotCoordinates(result);
                int id = result.getClassId();
                items.add(new Item(id, robotCoordinates));
            }
        }

        return items;
    }

    /**
     * Displays the detected items in telemetry.
     *
     * @param results List of detected items to display
     */
    private void displayResults(ArrayList<Item> results) {
        StringBuilder output = new StringBuilder();

        for (Item item : results) {
            output.append(item.toString()).append("\n");
        }

        telemetry.addData("Results", output.toString());
        telemetry.update();
    }

    /**
     * Transforms detector result using homography matrix to get robot-relative coordinates.
     *
     * @param result The detector result to transform
     * @return Robot-relative coordinates of the detected object
     */
    private Point calculateRobotCoordinates(LLResultTypes.DetectorResult result) {
        // Get angular coordinates
        double x_degrees = result.getTargetXDegrees();
        double y_degrees = result.getTargetYDegrees();

        // Convert to pixel coordinates using configurable resolution and FOV
        double x = (x_degrees / HORIZONTAL_FOV) * CAMERA_WIDTH + (CAMERA_WIDTH / 2.0);
        double y = (y_degrees / VERTICAL_FOV) * CAMERA_HEIGHT + (CAMERA_HEIGHT / 2.0);

        // Apply homography transformation
        double X_prime = H[0][0] * x + H[0][1] * y + H[0][2];
        double Y_prime = H[1][0] * x + H[1][1] * y + H[1][2];
        double W = H[2][0] * x + H[2][1] * y + H[2][2];

        // Convert to robot coordinates with offsets
        double x_robot = X_prime / W / PPI + HORIZONTAL_OFFSET;
        double y_robot = Y_prime / W / PPI + VERTICAL_OFFSET;

        return new Point(x_robot, y_robot);
    }
}

/**
 * Helper class to store results after transformation.
 */
class Item {
    // Detected Class ID
    private final int id;

    // Robot-relative position
    private final Point position;

    /**
     * Creates a new Item with ID and position.
     *
     * @param id The class ID of the detected object
     * @param position The position in robot-relative coordinates
     */
    public Item(int id, Point position) {
        this.id = id;
        this.position = position;
    }

    @Override
    public String toString() {
        return "Detected Item " + id + " at " + position.toString();
    }
}