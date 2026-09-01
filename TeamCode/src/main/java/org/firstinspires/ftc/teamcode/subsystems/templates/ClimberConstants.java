package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Constants and hardware configuration for the ClimberSubsystem.
 *
 * Overview:
 * Defines hardwareMap configuration names and preset power limits for
 * the quad continuous-rotation servo (CRServo) climbing winch mechanism.
 *
 * Hardware Configuration:
 * - Left Climber Outer Servo:  "leftLClimber"
 * - Left Climber Inner Servo:  "leftRClimber"
 * - Right Climber Inner Servo: "rightLClimber"
 * - Right Climber Outer Servo: "rightRClimber"
 *
 * Power Presets:
 * - Extension Power:   1.0 (Full Forward)
 * - Retraction Power: -1.0 (Full Reverse)
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
@Configurable
public class ClimberConstants {

    /** Private constructor to prevent instantiation. */
    private ClimberConstants() {}

    /** Hardware configuration name for the left inner climber CRServo. */
    public static String LEFT_SERVOR_NAME = "leftRClimber";

    /** Hardware configuration name for the right outer climber CRServo. */
    public static String RIGHT_SERVOR_NAME = "rightRClimber";

    /** Hardware configuration name for the left outer climber CRServo. */
    public static String LEFT_SERVOL_NAME = "leftLClimber";

    /** Hardware configuration name for the right inner climber CRServo. */
    public static String RIGHT_SERVOL_NAME = "rightLClimber";

    /** Preset motor/servo power for extension / climbing up (range: -1.0 to 1.0). */
    public static double POWER_UP = 1.0;

    /** Preset motor/servo power for retraction / pulling down (range: -1.0 to 1.0). */
    public static double POWER_DOWN = -1.0;
}