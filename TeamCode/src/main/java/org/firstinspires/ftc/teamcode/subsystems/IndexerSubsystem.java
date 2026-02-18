package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DistanceSensor; // IMPORTANTE: Import do Sensor de Distância
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * The IndexerSubsystem is responsible for managing the game pieces within the robot's indexer.
 * It uses a variety of sensors to detect the presence and count of game pieces.
 */
public class IndexerSubsystem extends SubsystemBase {

    private final TelemetryManager telemetry;

    // Sensor de Saída (Cor)
    private final NormalizedColorSensor sensorColor;

    // Sensor de Entrada (Distância 2M)
    private final DistanceSensor entrySensor;

    private int pieceCount = 0;

    private final float[] hsvValues = new float[3];

    private boolean previousEntryState = false;
    private long lastBallCountTime = 0;
    private static final long DEBOUNCE_DELAY_MS = 300;

    /**
     * Constructs a new IndexerSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     * @param telemetry   The telemetry manager for logging.
     */
    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        sensorColor = hardwareMap.get(NormalizedColorSensor.class, IndexerConstants.EXIT_SENSOR_NAME);

        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);

        if (sensorColor instanceof SwitchableLight) {
            ((SwitchableLight) sensorColor).enableLight(false);
        }
    }

    /**
     * This method is called periodically to update the subsystem's state and telemetry.
     */
    @Override
    public void periodic() {
        sensorColor.setGain(IndexerConstants.SENSOR_GAIN);
        NormalizedRGBA colors = sensorColor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        boolean currentEntryState = getEntrySensor();

        if (currentEntryState && !previousEntryState && (System.currentTimeMillis() - lastBallCountTime > DEBOUNCE_DELAY_MS)) {
            if (pieceCount < IndexerConstants.MAX_PIECE_CAPACITY) {
                pieceCount++;
            }
            lastBallCountTime = System.currentTimeMillis();
        }
        previousEntryState = currentEntryState;

        // Telemetria
        telemetry.addData("Indexer Pieces", pieceCount);

        // Info da Saída
        telemetry.addData("Exit Hue", hsvValues[0]);
        telemetry.addData("Exit Sat", hsvValues[1]);
        telemetry.addData("Exit Val", hsvValues[2]);
        telemetry.addData("Exit Triggered", getExitSensor());

        telemetry.addData("Entry Dist (CM)", entrySensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Entry Triggered", getEntrySensor());
    }

    /**
     * Gets the state of the exit sensor.
     * @return True if the sensor is triggered, false otherwise.
     */
    public boolean getExitSensor() {
        return (hsvValues[0] > IndexerConstants.HUE_OFFSET ||
                hsvValues[1] > IndexerConstants.SATURATION_OFFSET ||
                hsvValues[2] > IndexerConstants.VALUE_OFFSET);
    }

    /**
     * Gets the state of the ENTRY sensor.
     * @return True if the sensor is triggered, false otherwise.
     */
    public boolean getEntrySensor() {
        return entrySensor.getDistance(DistanceUnit.CM) < IndexerConstants.ENTRY_DISTANCE_CM;
    }

    /**
     * Gets the current number of game pieces in the indexer.
     * @return The number of game pieces.
     */
    public int getPieceCount() {
        return pieceCount;
    }

    /**
     * Checks if the indexer has any game pieces.
     * @return True if the indexer contains at least one piece, false otherwise.
     */
    public boolean hasPieces() {
        return pieceCount > 0;
    }

    /**
     * Checks if the indexer is full.
     * @return True if the indexer is at maximum capacity, false otherwise.
     */
    public boolean isFull() {
        return pieceCount >= IndexerConstants.MAX_PIECE_CAPACITY;
    }

    /**
     * Sets the number of game pieces in the indexer.
     * @param count The new piece count.
     */
    public void setPieceCount(int count) {
        this.pieceCount = count;
    }
}