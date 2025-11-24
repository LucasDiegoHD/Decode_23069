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
    public static String ENTRY_SENSOR_NAME = "sensorInput";
    /** The hardware map name for the exit sensor (e.g., a color/distance sensor). */
    public static String EXIT_SENSOR_NAME = "sensor_color_distance";

    /** The maximum number of game pieces the robot can hold. */
    public static int MAX_PIECE_CAPACITY = 3;
    /** The distance threshold in centimeters for the exit sensor to be considered triggered. */
    public static double DISTANCE_OFFSET = 2.4;
    public static int COLOR_OFFSET = 200;
}
