package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants for the IndexerSubsystem.
 */
@Configurable
public class IndexerConstants {
    /**
     * Private constructor to prevent instantiation.
     */
    private IndexerConstants() {}

    /**
     * The hardware map name for the entry sensor (e.g., a beam break sensor).
     */
    public static String ENTRY_SENSOR_NAME = "sensorEntry";
    /** The hardware map name for the exit sensor (e.g., a color/distance sensor). */
    public static String EXIT_SENSOR_NAME = "sensor_color_distance";
    public static final String MIDDLE_SENSOR_NAME = "middleSensor";

    /** The maximum number of game pieces the robot can hold. */
    public static int MAX_PIECE_CAPACITY = 3;
    /** The distance threshold in centimeters for the exit sensor to be considered triggered. */
    public static double EXIT_DISTANCE_CM = 10.0;
    public static double ENTRY_DISTANCE_CM = 10.0;
    public static int MIDDLE_LIGHT_THRESHOLD = 160;
}
