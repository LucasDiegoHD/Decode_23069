package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    private boolean isShooting = false;
    // Fica lendo apenas a cada 50ms para não engasgar o Loop do robô
    private long lastI2cReadTime = 0;
    private static final long I2C_READ_INTERVAL_MS = 50;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);
    }

    @Override
    public void periodic() {
        // --- PROTEÇÃO ANTI-LAG DO I2C ---
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastI2cReadTime >= I2C_READ_INTERVAL_MS) {
            currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
            currentExitDist = exitSensor.getDistance(DistanceUnit.CM);
            lastI2cReadTime = currentTime;
        }

        boolean currentEntryState = getEntrySensor();
        boolean currentExitState = getExitSensor();

        if (!isInitialized) {
            if (currentExitState && currentEntryState) {
                pieceCount = IndexerConstants.MAX_PIECE_CAPACITY;
            }
            if (currentExitState ||  currentEntryState) {
                pieceCount = 1;
            }
            else {
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
        if (!currentExitState && previousExitState && isShooting && (System.currentTimeMillis() - lastExitCountTime > DEBOUNCE_DELAY_MS)) {
            if (pieceCount > 0) {
                pieceCount--;
            }
            lastExitCountTime = System.currentTimeMillis();
        }

        previousEntryState = currentEntryState;
        previousExitState = currentExitState;

        telemetry.addData("Indexer Pieces", pieceCount);
        telemetry.addData("Is Shooting?", isShooting);

        telemetry.addData("Entry Dist (CM)", currentEntryDist);
        telemetry.addData("Entry Triggered", currentEntryState);

        telemetry.addData("Exit Dist (CM)", currentExitDist);
        telemetry.addData("Exit Triggered", currentExitState);
    }

    public void setShootingState(boolean state) {
        this.isShooting = state;
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