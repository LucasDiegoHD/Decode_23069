package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants for the ElevatorSubsystem.
 */
@Configurable
public class ElevatorConstants {

    // --- Hardware Map Names ---
    /** The hardware map name for the left lift motor. */
    public static String LEFT_MOTOR_NAME = "leftLift";
    /** The hardware map name for the right lift motor. */
    public static String RIGHT_MOTOR_NAME = "rightLift";

    // --- PID Constants (Error Correction) ---
    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.001;

    // --- Feedforward Constants (Physics: Gravity and Friction) ---
    /** Minimum power required to overcome static friction. */
    public static double kS = 0.01;
    /** Constant power required to hold the elevator in the air against gravity. */
    public static double kG = 0.1;
    /** Power required to maintain a constant velocity. */
    public static double kV = 0.02;
    /** Power required to accelerate the mass. */
    public static double kA = 0.005;

    // --- Motion Profile Limits (Smoothness and Speed) ---
    /** Maximum velocity in ticks per second. */
    public static double MAX_VELOCITY = 2000.0;
    /** Maximum acceleration in ticks per second squared. */
    public static double MAX_ACCELERATION = 1500.0;

    // --- Soft Limits (Safety Constraints) ---
    /** Minimum safe position in ticks to prevent bottoming out. */
    public static int MIN_POSITION_TICKS = 0;
    /** Maximum safe position in ticks to prevent overextension. */
    public static int MAX_POSITION_TICKS = 3000;

    // --- Target Positions (Setpoints) ---
    /** Target position for a fully retracted elevator. */
    public static int POS_RETRACTED = 0;
    /** Target position for scoring in the low basket. */
    public static int POS_LOW_BASKET = 1000;
    /** Target position for scoring in the high basket. */
    public static int POS_HIGH_BASKET = 2500;
}