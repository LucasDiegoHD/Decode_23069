package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ClimberConstants {
    public static String LEFT_SERVOR_NAME = "leftRClimber";
    public static String RIGHT_SERVOR_NAME = "rightRClimber";
    public static String LEFT_SERVOL_NAME = "leftLClimber";
    public static String RIGHT_SERVOL_NAME = "rightLClimber";
    // Potência de subida (1.0 = 100% pra frente)
    public static double POWER_UP = 1.0;

    // Potência de descida (-1.0 = 100% pra trás)
    public static double POWER_DOWN = -1.0;
}