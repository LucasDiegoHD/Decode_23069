package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants for the ShooterSubsystem.
 */
@Configurable
public class ShooterConstants {


    public static double kP = 10;


    public static double kI = 0.0;


    public static double kD = 0.0;


    public static double kF = 0.05;


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
    public static double TARGET_VELOCITY_SHORT = 3800;
    /** The target velocity in RPM for long shots. */
    public static double TARGET_VELOCITY_LONG = 3700;
    /** The acceptable error margin for the shooter's target velocity in RPM. */
    public static double VELOCITY_TOLERANCE = 30;

    /** The minimum position of the hood servo. */
    public static double MINIMUM_HOOD = 0.0;
    /** The maximum position of the hood servo. */
    public static double MAXIMUM_HOOD = 0.9;
    /** The amount to increment or decrement the hood position. */
    public static double HOOD_INCREMENT = 0.05;

    /** Ganho de compensação: quanto o hood sobe por cada 1 RPM de erro. */
    public static double K_HOOD_COMPENSATION = 0.00045;
    /** Tolerância de cadência agressiva (85%) para tiros longos. */
    public static double CADENCE_TOLERANCE_PERCENT = 0.85;

    // --- TIMINGS ---
    public static double INTAKE_TIME_TO_SHOOT = 350;
    /** The time in milliseconds to wait after triggering before shooting. */
    public static double TRIGGER_TIMER_TO_SHOOT = 500;
    public static double TRIGGER_TIMER_TRIGGERING = 1300;

    /**
     * Equations for hood
     * y = Hood Angle
     * x = Distance (from Limelight)
     * Formula: y = 0.2799x^3 - 1.7289x^2 + 3.7225x - 2.1911
     */
    public static double HOOD_N0 = -2.1211;
    public static double HOOD_N1 = 3.7225;
    public static double HOOD_N2 = -1.7289;
    public static double HOOD_N3 = 0.2799;

    /**
     * Equations for RPM
     * y = RPM
     * x = Distance (from Limelight)
     * Formula: y = 842.9828x + 2790.7711
     */
    public static double RPM_N0 = 2640.7711;
    public static double RPM_N1 = 842.9828;
    public static double ANGLE_KP = 0.75;
    public static double ANGLE_KI = 0.01;
    public static double ANGLE_KD = 0.075;
    public static double ANGLE_KF = 0.1;
    public static double ANGLE_TOLERANCE = 0.25;
}