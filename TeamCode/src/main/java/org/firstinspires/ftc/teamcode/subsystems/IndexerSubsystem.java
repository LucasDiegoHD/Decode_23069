package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * The IndexerSubsystem is responsible for managing the game pieces within the robot's indexer.
 * It passively keeps track of the inventory using entry and exit sensors.
 */
public class IndexerSubsystem extends SubsystemBase {

    private final TelemetryManager telemetry;

    private final DistanceSensor exitSensor;
    private final DistanceSensor entrySensor;

    private int pieceCount = 0;

    private boolean previousEntryState = false;
    private long lastEntryCountTime = 0;

    private boolean previousExitState = false;
    private long lastExitCountTime = 0;

    private static final long DEBOUNCE_DELAY_MS = 100;

    private double currentEntryDist = 23069.0;
    private double currentExitDist = 23069.0;

    private boolean isInitialized = false;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);
    }

    @Override
    public void periodic() {
        currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
        currentExitDist = exitSensor.getDistance(DistanceUnit.CM);

        boolean currentEntryState = getEntrySensor();
        boolean currentExitState = getExitSensor();

        if (!isInitialized) {
            if (currentEntryState) {
                pieceCount = IndexerConstants.MAX_PIECE_CAPACITY;
            } else {
                pieceCount = 0;
            }
            previousEntryState = currentEntryState;
            previousExitState = currentExitState;
            isInitialized = true;
        }

        if (currentEntryState && !previousEntryState && (System.currentTimeMillis() - lastEntryCountTime > DEBOUNCE_DELAY_MS)) {
            if (pieceCount < IndexerConstants.MAX_PIECE_CAPACITY) {
                pieceCount++;
            }
            lastEntryCountTime = System.currentTimeMillis();
        }

        if (!currentExitState && previousExitState && (System.currentTimeMillis() - lastExitCountTime > DEBOUNCE_DELAY_MS)) {
            if (pieceCount > 0) {
                pieceCount--;
            }
            lastExitCountTime = System.currentTimeMillis();
        }

        previousEntryState = currentEntryState;
        previousExitState = currentExitState;

        telemetry.addData("Indexer Pieces", pieceCount);
        telemetry.addData("Is Initialized?", isInitialized);

        telemetry.addData("Entry Dist (CM)", currentEntryDist);
        telemetry.addData("Entry Triggered", currentEntryState);

        telemetry.addData("Exit Dist (CM)", currentExitDist);
        telemetry.addData("Exit Triggered", currentExitState);
    }

    public boolean getExitSensor() {
        return currentExitDist < IndexerConstants.EXIT_DISTANCE_CM;
    }

    public boolean getEntrySensor() {
        return currentEntryDist < IndexerConstants.ENTRY_DISTANCE_CM;
    }

    public int getPieceCount() {
        return pieceCount;
    }

    public boolean hasPieces() {
        return pieceCount > 0;
    }

    public boolean isFull() {
        return pieceCount >= IndexerConstants.MAX_PIECE_CAPACITY;
    }

    public void setPieceCount(int count) {
        if (count < 0) count = 0;
        if (count > IndexerConstants.MAX_PIECE_CAPACITY) count = IndexerConstants.MAX_PIECE_CAPACITY;
        this.pieceCount = count;
    }
}