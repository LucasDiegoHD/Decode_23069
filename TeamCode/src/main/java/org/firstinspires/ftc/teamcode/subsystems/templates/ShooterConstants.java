package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration, PIDF constants, and polynomial regression parameters for the ShooterSubsystem.
 *
 * Overview:
 * Defines the closed-loop velocity PIDF gains, feedforward voltage compensation constants,
 * servo travel limits, shot boost durations, and regression coefficients mapping physical
 * target distances (in meters) to optimal flywheel RPM and hood servo launch angles.
 *
 * Flywheel Control Strategy:
 * Uses a combined feedforward and feedback voltage-compensated controller:
 *   Voltage Compensation = 12.0 / battery_voltage
 *   Feedforward = (kS * sign(targetRPM) + kV * targetRPM) * Voltage Compensation
 *   Feedback = PIDF.calculate(currentRPM, targetRPM)
 *   ShotBoost = kF_SHOT_BOOST * Voltage Compensation (during active gate trigger)
 *
 * Polynomial Range Equations:
 * - Hood Angle (0.0 to 1.0): y = HOOD_N0 + HOOD_N1*x + HOOD_N2*x^2 + HOOD_N3*x^3
 *   (where x is target distance in meters)
 * - Flywheel RPM: y = RPM_N0 + RPM_N1*x + RPM_N2*x^2
 *   (where x is target distance in meters)
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
@Configurable
public class ShooterConstants {

    /** Private constructor to prevent instantiation. */
    private ShooterConstants() {}

    // --- Closed-Loop Velocity PIDF and Feedforward Constants ---

    /** Proportional gain for flywheel velocity PID controller. */
    public static double kP = 30;

    /** Integral gain for flywheel velocity PID controller. */
    public static double kI = 0.0;

    /** Derivative gain for flywheel velocity PID controller. */
    public static double kD = 0.00001;

    /** Static feedforward gain for basic PIDF controller. */
    public static double kF = 0.05;

    /** Static friction overcoming feedforward gain (kS) for flywheel model. */
    public static double kS = 0.2;

    /** Velocity proportional feedforward gain (kV) for flywheel model. */
    public static double kV = 0.003;

    /** Feedforward boost power applied during active firing to counteract piece drag. */
    public static double kF_SHOT_BOOST = 0.3;

    /** Duration in milliseconds for which the shot boost feedforward is active. */
    public static double SHOT_BOOST_DURATION = 400;

    // --- Hardware Map Identifiers ---

    /** Hardware configuration name for the right flywheel motor (DcMotorEx). */
    public static String RSHOOTER_MOTOR_NAME = "rightShooterMotor";

    /** Hardware configuration name for the left flywheel motor (DcMotorEx). */
    public static String LSHOOTER_MOTOR_NAME = "leftShooterMotor";

    /** Hardware configuration name for the trigger/indexer feeder motor. */
    public static String TRIGGER_MOTOR_NAME = "triggerMotor";

    /** Hardware configuration name for the left hood angle servo. */
    public static String HOOD_SERVO_LEFT_NAME = "Hood_ServoL";

    /** Hardware configuration name for the right hood angle servo. */
    public static String HOOD_SERVO_RIGHT_NAME = "Hood_ServoR";

    // --- Flywheel Motor Resolution and Presets ---

    /**
     * Encoder resolution in ticks per revolution.
     * 28.0 ticks/rev corresponds to direct-drive 1:1 motors (e.g., GoBilda 6000 RPM Yellowjacket).
     */
    public static final double TICKS_PER_REV = 28.0;

    /** Preset target velocity in RPM for short-range shots. */
    public static double TARGET_VELOCITY_SHORT = 2500;

    /** Preset target velocity in RPM for long-range shots. */
    public static double TARGET_VELOCITY_LONG = 2900;

    /** Acceptable error margin in RPM for steady-state velocity checks. */
    public static double VELOCITY_TOLERANCE = 30;

    // --- Hood Servo Limits and Micro-Adjustments ---

    /** Minimum allowable normalized position for the hood angle servos (steepest trajectory). */
    public static double MINIMUM_HOOD = 0.0;

    /** Maximum allowable normalized position for the hood angle servos (flattest trajectory). */
    public static double MAXIMUM_HOOD = 0.9;

    /** Increment step size for manual/trim hood position adjustments. */
    public static double HOOD_INCREMENT = 0.05;

    /** Dynamic hood compensation gain: hood position adjustment per 1 RPM of undershoot. */
    public static double K_HOOD_COMPENSATION = 0.0005;

    /** Dynamic anti-overshoot gain: hood position adjustment per 1 RPM of overshoot. */
    public static double K_ANTI_OVERSHOOT = 0.00025;

    /** Angular velocity feedforward scaling gain for drive-to-aim kinematics. */
    public static double K_OMEGA = 0.05;

    /** Ratio of target RPM required before firing cadence allows launching. */
    public static double CADENCE_TOLERANCE_PERCENT = 0.94;

    // --- State Machine Timers (in milliseconds) ---

    /** Delay in milliseconds before firing to ensure trigger mechanism readiness. */
    public static double TRIGGER_TIMER_TO_SHOOT = 400;

    /** Maximum active triggering time window in milliseconds for feeding a piece. */
    public static double TRIGGER_TIMER_TRIGGERING = 800;

    /** Cooldown duration between consecutive shots in multi-shot bursts. */
    public static double DELAY_BETWEEN_SHOTS_MS = 10;

    /** Follow-through feeding duration in milliseconds after a piece clears the sensor. */
    public static double TRIGGER_FOLLOW_THROUGH_MS = 350;

    // --- Hood Polynomial Coefficients: y = N0 + N1*x + N2*x^2 + N3*x^3 ---

    /** Constant term (Degree 0) for distance-to-hood polynomial curve. */
    public static double HOOD_N0 = 0.84;

    /** Linear coefficient (Degree 1) for distance-to-hood polynomial curve. */
    public static double HOOD_N1 = 0.0;

    /** Quadratic coefficient (Degree 2) for distance-to-hood polynomial curve. */
    public static double HOOD_N2 = 0.0;

    /** Cubic coefficient (Degree 3) for distance-to-hood polynomial curve. */
    public static double HOOD_N3 = 0.0;

    // --- Flywheel RPM Polynomial Coefficients: y = N0 + N1*x + N2*x^2 ---

    /** Constant term (Degree 0) for distance-to-RPM polynomial curve. */
    public static double RPM_N0 = 2630.5;

    /** Linear coefficient (Degree 1) for distance-to-RPM polynomial curve. */
    public static double RPM_N1 = -521.9;

    /** Quadratic coefficient (Degree 2) for distance-to-RPM polynomial curve. */
    public static double RPM_N2 = 171.8;

    // --- Turret / Aim Alignment PID Constants ---

    /** Proportional gain for heading kinematic aiming controller. */
    public static double ANGLE_KP = 1.05;

    /** Integral gain for heading kinematic aiming controller. */
    public static double ANGLE_KI = 0.025;

    /** Derivative gain for heading kinematic aiming controller. */
    public static double ANGLE_KD = 0.1;

    /** Feedforward gain for heading kinematic aiming controller. */
    public static double ANGLE_KF = 0.15;

    /** Heading error deadband tolerance in degrees for kinematic lock. */
    public static double ANGLE_TOLERANCE = 0.06;
}