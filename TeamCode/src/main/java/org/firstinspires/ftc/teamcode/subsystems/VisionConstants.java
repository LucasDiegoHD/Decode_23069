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

    /** The height of the camera from the ground in meters. */
    public static double CAMERA_HEIGHT_METERS = 0.24554;
    /** The height of the target from the ground in meters. */
    public static double TARGET_HEIGHT_METERS = 0.72;
    /** The pitch of the camera in degrees. */
    public static double CAMERA_PITCH_DEGREES = 24.44;
    public static double UPDATE_POSE_VISION_TIMEOUT = 2000;
    public static double LONGEST_HOOD = 0.7;
    public static double LONGEST_RPM = 4700;
    public static double LONGEST_DISTANCE = 2.2;
    // ---- Tempo ----
    public static double UPDATE_INTERVAL_SECONDS = 2.0;

    // ---- Pesos ----
    public static double BASE_WEIGHT_ODOMETRY = 0.8;
    public static double BASE_WEIGHT_LIMELIGHT = 0.2;

    // ---- Filtro EMA (suavização da limelight) ----
    public static double EMA_ALPHA = 0.4;  // 0 = muito suave, 1 = sem suavização

    // ---- Tolerância para rejeição de outlier ----
    public static double MAX_ALLOWED_JUMP = 30.0;  // distância máxima em cm

    // ---- Peso adaptativo ----
    public static boolean ENABLE_ADAPTIVE_WEIGHT = true;
    public static double ADAPTIVE_ERROR_SCALE = 50.0; // quanto maior, menos altera peso

}
