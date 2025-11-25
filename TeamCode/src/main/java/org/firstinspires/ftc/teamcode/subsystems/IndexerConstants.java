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
    public static double HUE_OFFSET = 100;
    public static double SATURATION_OFFSET = 0.5;
    public static double VALUE_OFFSET = 0.001;
    public static float SENSOR_GAIN = 2;
}
