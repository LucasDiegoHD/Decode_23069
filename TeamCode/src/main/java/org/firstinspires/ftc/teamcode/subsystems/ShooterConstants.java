package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants for the ShooterSubsystem.
 */
@Configurable
public class ShooterConstants {


    public static double kP = 30;


    public static double kI = 0.0;


    public static double kD = 0.00001;


    public static double kF = 0.05;

    public static double kS = 0.2 ;

    public static double kV = 0.003;

    public static double kF_SHOT_BOOST = 0.3;
    public static double SHOT_BOOST_DURATION = 400;


    /** The hardware map name for the right shooter motor. */
    public static String RSHOOTER_MOTOR_NAME = "rightShooterMotor";
    /** The hardware map name for the left shooter motor. */
    public static String LSHOOTER_MOTOR_NAME = "leftShooterMotor";
    /** The hardware map name for the trigger motor. */
    public static String TRIGGER_MOTOR_NAME = "triggerMotor";
    /** The hardware map name for the hood servo. */
    public static String HOOD_SERVO_LEFT_NAME = "Hood_ServoL";
    public static String HOOD_SERVO_RIGHT_NAME = "Hood_ServoR";

    /**
     * The number of encoder ticks per revolution for the shooter motors.
     * 28.0 ticks implies a 1:1 ratio motor (e.g., GoBilda 6000 RPM Series).
     */
    public static final double TICKS_PER_REV = 28.0;

    /** The target velocity in RPM for short shots. */
    public static double TARGET_VELOCITY_SHORT = 2800;
    /** The target velocity in RPM for long shots. */
    public static double TARGET_VELOCITY_LONG = 3000;
    /** The acceptable error margin for the shooter's target velocity in RPM. */
    public static double VELOCITY_TOLERANCE = 30;

    /** The minimum position of the hood servo. */
    public static double MINIMUM_HOOD = 0.0;
    /** The maximum position of the hood servo. */
    public static double MAXIMUM_HOOD = 0.9;
    /** The amount to increment or decrement the hood position. */
    public static double HOOD_INCREMENT = 0.05;

    /** Ganho de compensação: quanto o hood sobe por cada 1 RPM de erro. */
    public static double K_HOOD_COMPENSATION = 0.00065;
    /** Tolerância de cadência para tiros longos. */
    public static double CADENCE_TOLERANCE_PERCENT = 0.90;

    // --- TIMINGS ---

    /** The time in milliseconds to wait after triggering before shooting. */
    public static double TRIGGER_TIMER_TO_SHOOT = 400;
    public static double TRIGGER_TIMER_TRIGGERING = 1000;
    public static double DELAY_BETWEEN_SHOTS_MS = 25;


    /**
     * Equations for hood
     * y = Hood Angle
     * x = Distance (from Vision/Limelight in Meters)
     * * Points used:
     * 0.75m -> 0.75
     * 1.00m -> 0.70
     * 1.50m -> 0.55
     * 2.00m -> 0.50
     * * Fit: Quadratic (Grau 2) - Muito preciso para esses pontos.
     * Formula: y = 0.12x^2 - 0.53x + 1.08
     */
    public static double HOOD_N0 = 0.90;
    public static double HOOD_N1 = 0.0;
    public static double HOOD_N2 = 0.0;
    public static double HOOD_N3 = 0.0;

    /**
     * Equations for RPM
     * y = RPM
     * x = Distance (from Vision/Limelight in Meters)
     * * Points used:
     * 0.75m -> 2500
     * 1.00m -> 2500
     * 1.50m -> 3800
     * 2.00m -> 3000
     * * Fit: Linear Approx
     * Formula: y = 434x + 2480
     */
    public static double RPM_N0 = 2154.3;
    public static double RPM_N1 = -303.7;
    public static double RPM_N2 = 142.1;

    public static double ANGLE_KP = 0.8;
    public static double ANGLE_KI = 0.015;
    public static double ANGLE_KD = 0.05;
    public static double ANGLE_KF = 0.125;
    public static double ANGLE_TOLERANCE = 0.1;
}