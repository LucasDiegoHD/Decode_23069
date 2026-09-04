package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * LED Indicator Subsystem Template
 *
 * Overview:
 * Controls visual status signaling on the robot via a PWM-controlled addressable LED controller
 * (such as a REV Blinkin or servo-driven LED strip driver). Provides real-time visual feedback
 * to drivers and field referees regarding game piece capacity, auto-alignment lock, and subsystem health.
 *
 * Hardware Configuration:
 * - LED Controller (Servo Port): "led_indicator"
 * - PWM Pulse Width Range: 500 us to 2500 us
 *
 * Color and Pattern Presets:
 * Output colors are selected by setting the normalized servo position corresponding
 * to the controller's internal lookup table (RED, GREEN, BLUE, YELLOW, WHITE, ORANGE, OFF).
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
public class LEDSubsystem {

    private final ServoImplEx led;
    private final IndexerSubsystem indexer;
    private double targetPosition = 0.0;
    private int lastBolas = -1;

    /** Preset PWM position for solid RED color indicator. */
    public static final double RED = 0.277;

    /** Preset PWM position for solid GREEN color indicator. */
    public static final double GREEN = 0.500;

    /** Preset PWM position for solid BLUE color indicator. */
    public static final double BLUE = 0.611;

    /** Preset PWM position to turn LED output OFF. */
    public static final double OFF = 0.0;

    /** Preset PWM position for solid WHITE color indicator. */
    public static final double WHITE = 1.0;

    /** Preset PWM position for solid ORANGE color indicator. */
    public static final double ORANGE = 0.333;

    /** Preset PWM position for solid YELLOW color indicator. */
    public static final double YELLOW = 0.338;

    /**
     * Constructs a new LEDSubsystem, configures extended PWM timing range (500–2500 us),
     * and sets initial state to RED.
     *
     * @param hardwareMap Robot hardware map for device retrieval.
     * @param indexer Indexer consultado para derivar a cor a partir da contagem de pecas.
     */
    public LEDSubsystem(HardwareMap hardwareMap, IndexerSubsystem indexer) {
        this.indexer = indexer;
        led = hardwareMap.get(ServoImplEx.class, "led_indicator");
        led.setPwmRange(new PwmControl.PwmRange(500, 2500));
        this.targetPosition = RED;
    }

    /**
     * Sets the desired LED output pattern or color position.
     *
     * @param position Normalized servo position (e.g., GREEN, YELLOW, etc.).
     */
    public void setPattern(double position) {
        this.targetPosition = position;
    }

    /**
     * Rotina continua. Mapeia a contagem de pecas do indexer para a cor e transmite a posicao
     * ativa ao controlador. Absorve o antigo {@code LedCommand}: a cor so e recalculada quando a
     * contagem muda, para nao reescrever a posicao a cada loop sem necessidade.
     */
    public void update() {
        int bolas = indexer.getPieceCount();

        if (bolas != lastBolas) {
            if (bolas == 0) {
                setPattern(OFF);
            } else if (bolas == 1) {
                setPattern(BLUE);
            } else if (bolas == 2) {
                setPattern(ORANGE);
            } else if (bolas >= 3) {
                setPattern(GREEN);
            }

            lastBolas = bolas;
        }

        led.setPosition(targetPosition);
    }
}