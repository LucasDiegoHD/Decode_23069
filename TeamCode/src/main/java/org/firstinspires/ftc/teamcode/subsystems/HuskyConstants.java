package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class HuskyConstants {
    private HuskyConstants() {}

    // --- Calibração da Câmera (CRÍTICO) ---
    // Fórmula para achar este valor: (Largura_Pixels_Lida * Distancia_Real_Inches) / Largura_Objeto_Real_Inches
    // Exemplo: Se a bola de 5" aparece com 50px de largura a 20" de distância: (50 * 20) / 5 = 200.
    public static double FOCAL_LENGTH_PIXELS = 200.0;
    public static double ARTIFACT_REAL_WIDTH_INCHES = 5.0; // Diâmetro da bola DECODE

    // --- Setpoints de Navegação ---
    public static double TARGET_DISTANCE_INCHES = 1.0; // Distância que o robô deve parar da bola
    public static double CENTER_X_PIXELS = 160.0;      // Centro horizontal da tela (320x240)

    // --- Ganhos PID (Alinhamento) ---
    public static double TURN_KP = 0.1;  // Força do giro por erro de pixel
    public static double TURN_KI = 0.0;
    public static double TURN_KD = 0.0001;

    // --- Ganhos PID (Aproximação) ---
    public static double DRIVE_KP = 0.1;  // Força do drive por erro de polegadas
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.001;

    // --- Limites de Segurança ---
    public static double MAX_DRIVE_SPEED = 0.6;
    public static double MAX_TURN_SPEED = 0.5;
    public static double DEADZONE_ALIGN_PIXELS = 5.0; // Margem de erro aceitável para X
    public static double DEADZONE_DIST_INCHES = 1.0;  // Margem de erro aceitável para Distância
    public static int COLOR_ID_PURPLE = 1;
    public static int COLOR_ID_GREEN = 2;

}
