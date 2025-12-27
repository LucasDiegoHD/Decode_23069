package org.firstinspires.ftc.teamcode.subsystems.indexer;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


/**
 * The IndexerSubsystem is responsible for managing the game pieces within the robot's indexer.
 * It uses a variety of sensors to detect the presence and count of game pieces.
 */
//@AutoLog
public class IndexerSubsystem extends SubsystemBase {

    private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final NormalizedColorSensor sensorColor;
    private int pieceCount = 0;

    private final boolean lastEntryState = false;
    private final boolean lastExitState = false;
    private final float[] hsvValues = new float[3];
    /**
     * Constructs a new IndexerSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     */
    public IndexerSubsystem(HardwareMap hardwareMap) {

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(NormalizedColorSensor.class, IndexerConstants.EXIT_SENSOR_NAME);

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
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


        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);
        telemetry.addData("Sensor", getExitSensor());
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
