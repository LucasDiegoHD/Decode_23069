package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PurePursuitConstants {

    // --- FORWARD PID (Controla distância frente/trás) ---
    public static double FWD_P = 0.1;
    public static double FWD_I = 0.0;
    public static double FWD_D = 0.0;

    // --- STRAFE PID (Controla erro lateral) ---
    public static double STR_P = 0.12;
    public static double STR_I = 0.0;
    public static double STR_D = 0.0;

    // --- HEADING PID (Controla o ângulo) ---
    public static double HEAD_P = 0.8;
    public static double HEAD_I = 0.0;
    public static double HEAD_D = 0.05;

    // --- PARÂMETROS GERAIS ---

    // Distância do "Círculo de Lookahead".
    // Maior = Curvas mais suaves (corta caminho).
    // Menor = Segue a linha mais fielmente (pode oscilar).
    public static double LOOKAHEAD_DISTANCE = 12.0;

    // O quão perto do ponto final o robô precisa estar para considerar "Terminado"
    public static double END_TOLERANCE = 1.0;

    // Velocidade máxima geral (0 a 1) para limitar o robô se necessário
    public static double MAX_SPEED = 1.0;
    // --- LIMITADORES DE ACELERAÇÃO (Evitar Derrapagem) ---
    // Valor 2.0 = O motor leva 0.5s para ir de 0 a potência máxima (1.0)
    // Valor 1.0 = O motor leva 1.0s para ir de 0 a potência máxima (1.0)
    public static double ACCEL_FWD = 2.5;
    public static double ACCEL_STR = 2.0;
    public static double ACCEL_TURN = 3.0;
}