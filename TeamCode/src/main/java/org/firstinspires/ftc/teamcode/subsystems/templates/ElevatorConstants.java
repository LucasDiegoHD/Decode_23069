package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ElevatorConstants {

    // Nomes no Hardware Map
    public static String LEFT_MOTOR_NAME = "leftLift";
    public static String RIGHT_MOTOR_NAME = "rightLift";

    // PID Simples (Comece com kP muito baixo, ex: 0.005, e suba aos poucos)
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0001;

    // kF (Feedforward Simples): A potência mínima apenas para a gravidade não puxar a gaveta para baixo.
    // Para Viper Slides com Yellow Jackets, esse valor costuma ser bem baixo (ex: 0.05 a 0.1).
    public static double kF = 0.1;

    // Soft Limits
    public static int MIN_POSITION_TICKS = 0;
    // Ajuste este valor limite de acordo com o limite físico do seu Viper Slide de 4 estágios.
    public static int MAX_POSITION_TICKS = 3000;

    // Posições Alvo
    public static int POS_RETRACTED = 0;
    public static int POS_LOW_BASKET = 1000;
    public static int POS_HIGH_BASKET = 2500;
    public static int MANUAL_SPEED_TICKS = 50;
}