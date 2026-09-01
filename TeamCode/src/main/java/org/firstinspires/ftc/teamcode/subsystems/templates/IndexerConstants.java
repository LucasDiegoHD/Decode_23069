package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants and sensor thresholds for the IndexerSubsystem.
 *
 * Overview:
 * Defines hardware configuration names, physical distance thresholds, ambient
 * light triggers, and capacity limits for the 3-sensor multi-zone piece tracking system.
 *
 * Hardware Configuration:
 * - Entry Distance Sensor:          "sensorEntry" (I2C Bus)
 * - Middle REV Color/Light Sensor:  "middleSensor" (I2C Bus)
 * - Exit Distance/Color Sensor:     "sensor_color_distance" (I2C Bus)
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
@Configurable
public class IndexerConstants {

    /** Private constructor to prevent instantiation. */
    private IndexerConstants() {}

    // --- Hardware Map Identifiers ---

    /** Hardware configuration name for the entry distance sensor. */
    public static String ENTRY_SENSOR_NAME = "sensorEntry";

    /** Hardware configuration name for the exit color/distance sensor. */
    public static String EXIT_SENSOR_NAME = "sensor_color_distance";

    /** Hardware configuration name for the middle REV Color Sensor V3. */
    public static final String MIDDLE_SENSOR_NAME = "middleSensor";

    // --- Capacity and Threshold Settings ---

    /** Maximum piece capacity supported by the conveyor indexer hopper. */
    public static int MAX_PIECE_CAPACITY = 3;

    /** Distance threshold in centimeters for the exit sensor to detect a piece. */
    public static double EXIT_DISTANCE_CM = 10.0;

    /** Distance threshold in centimeters for the entry sensor to detect an incoming piece. */
    public static double ENTRY_DISTANCE_CM = 10.0;

    /** Alpha/ambient light intensity threshold for the middle REV Color Sensor to detect a piece. */
    public static int MIDDLE_LIGHT_THRESHOLD = 160;
}
