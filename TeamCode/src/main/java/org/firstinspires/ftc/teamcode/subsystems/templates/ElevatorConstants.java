package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ElevatorConstants {
    public static String LEFT_MOTOR_NAME = "leftLift";
    public static String RIGHT_MOTOR_NAME = "rightLift";
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0001;
    public static double kF = 0.1;

    // Soft Limits
    public static int MIN_POSITION_TICKS = -100;
    public static int MAX_POSITION_TICKS = 100;

    // Posições Alvo
    public static int POS_RETRACTED = 0;
    public static int POS_LOW_BASKET = 1000;
    public static int POS_HIGH_BASKET = 2500;
    public static int MANUAL_SPEED_TICKS = 250;
}