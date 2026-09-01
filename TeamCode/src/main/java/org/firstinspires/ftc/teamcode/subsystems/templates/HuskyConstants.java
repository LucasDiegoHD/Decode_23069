package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration and calibration constants for the HuskySubsystem.
 *
 * Overview:
 * Defines optical calibration parameters, target setpoints, color classification IDs,
 * velocity limits, and PID constants for visual artifact tracking and intake alignment
 * using the DFRobot HuskyLens smart camera.
 *
 * Optical Distance Estimation Formula:
 *   Distance (inches) = (Artifact Width * Focal Length) / Bounding Box Width (pixels)
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
@Configurable
public class HuskyConstants {

    /** Private constructor to prevent instantiation. */
    private HuskyConstants() {}

    // --- Camera Calibration ---

    /**
     * Estimated focal length in pixels (for 320x240 resolution).
     * Calibration formula: (Pixel_Width * Known_Distance_Inches) / Real_Object_Width_Inches
     */
    public static double FOCAL_LENGTH_PIXELS = 200.0;

    /** Real physical diameter of the game artifact ball (in inches). */
    public static double ARTIFACT_REAL_WIDTH_INCHES = 5.0;

    // --- Navigation and Alignment Setpoints ---

    /** Target stopping distance from the artifact during autonomous approach (inches). */
    public static double TARGET_DISTANCE_INCHES = 5.0;

    /** Horizontal center pixel coordinate for 320x240 camera resolution. */
    public static double CENTER_X_PIXELS = 160.0;

    /** Deadband pixel threshold for centering the object before disabling turn PID. */
    public static double DEADZONE_ALIGN_PIXELS = 20.0;

    // --- PID Coefficients ---

    /** Proportional gain for heading/turn alignment PID. */
    public static double TURN_KP = 0.002;

    /** Integral gain for heading/turn alignment PID. */
    public static double TURN_KI = 0.0;

    /** Derivative gain for heading/turn alignment PID. */
    public static double TURN_KD = 0.0;

    /** Proportional gain for distance approach PID. */
    public static double DRIVE_KP = 0.5;

    /** Integral gain for distance approach PID. */
    public static double DRIVE_KI = 0.0;

    /** Derivative gain for distance approach PID. */
    public static double DRIVE_KD = 0.00;

    // --- HuskyLens Color IDs ---

    /** Primary HuskyLens algorithm color ID trained for Purple artifacts. */
    public static int COLOR_ID_PURPLE = 2;

    /** Primary HuskyLens algorithm color ID trained for Green artifacts. */
    public static int COLOR_ID_GREEN = 1;

    /** Secondary / Alternate color ID for Purple artifacts under varied lighting. */
    public static int COLOR_ID_PURPLE2 = 3;

    /** Secondary / Alternate color ID for Green artifacts under varied lighting. */
    public static int COLOR_ID_GREEN2 = 4;

    // --- Velocity and Approach Thresholds ---

    /** Distance threshold (in inches) beyond which maximum "turbo" drive speed is applied. */
    public static final double TURBO_THRESHOLD = 12.0;

    /** Maximum drive power applied in turbo approach mode. */
    public static final double TURBO_SPEED = -0.85;

    /** Minimum drive power applied to overcome static friction during PID approach. */
    public static final double MIN_DRIVE_SPEED_PID = 0.15;

    /** Duration (in milliseconds) to maintain forward blind drive after target is lost at close range. */
    public static final long BLIND_DURATION_MS = 1500;
}
