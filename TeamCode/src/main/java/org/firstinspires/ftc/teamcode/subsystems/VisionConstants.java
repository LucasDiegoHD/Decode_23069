package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants for the VisionSubsystem.
 */
@Configurable
public class VisionConstants {
    /**
     * Private constructor to prevent instantiation.
     */
    private VisionConstants() {}

    /**
     * The proportional gain for the turn controller.
     */
    public static double TURN_KP = 0.012;
    /** The integral gain for the turn controller. */
    public static double TURN_KI = 0.15;
    /** The derivative gain for the turn controller. */
    public static double TURN_KD = 0.0035;
    /** The feedforward gain for the turn controller. */
    public static double TURN_KF = 0.1;


    public static double LONGEST_HOOD = 0.7;
    public static double LONGEST_RPM = 4500;
    public static double LONGEST_DISTANCE = 2.2;
    // ---- Tempo ----
    // distância máxima em METROS
    public static double MAX_DELTA_METERS = 1.0; // default = 1m

    // Pesos para fusão
    public static double ODOMETRY_WEIGHT = 0.8;
    public static double LIMELIGHT_WEIGHT = 0.2;

    // conversão
    public static final double METERS_TO_INCHES = 39.3700787;

}
