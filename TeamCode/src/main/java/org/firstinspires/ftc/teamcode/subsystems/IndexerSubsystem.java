package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexerSubsystem extends SubsystemBase {

    private final TelemetryManager telemetry;

    private final DistanceSensor exitSensor;
    private final DistanceSensor entrySensor;
    private final RevColorSensorV3 middleSensor;

    private int pieceCount = 0;
    private boolean isShooting = false;
    private long shootingEndTime = 0;
    private static final long POST_SHOOT_PROTECTION_MS = 500;
    private static final long RISE_DEBOUNCE_MS = 200;
    private static final long FALL_DEBOUNCE_MS = 80;

    private double currentEntryDist = 23069.0;
    private double currentExitDist = 23069.0;
    private int currentMiddleLight = 0;

    private boolean isInitialized = false;
    private long startTime = 0;
    private static final long INIT_DELAY_MS = 200;

    private int lastInferred = 0;
    private long lastRiseTime = 0;
    private long lastFallTime = 0;
    private int lastFallInferred = -1;
    private boolean previousExitActive = false;
    private long lastExitFallTime = 0;
    private static final long EXIT_FALL_DEBOUNCE_MS = 100;
    private long lastExitReadTime = 0;
    private long lastEntryReadTime = 0;
    private long lastMiddleReadTime = 0;
    private static final long EXIT_READ_INTERVAL_MS = 150;
    private static final long ENTRY_READ_INTERVAL_MS = 150;
    private static final long MIDDLE_READ_INTERVAL_MS = 150;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);
        middleSensor = hardwareMap.get(RevColorSensorV3.class, IndexerConstants.MIDDLE_SENSOR_NAME);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void periodic() {
        long tempoInicio = System.currentTimeMillis();

        if (!isInitialized && tempoInicio - startTime < INIT_DELAY_MS) {
            currentExitDist = exitSensor.getDistance(DistanceUnit.CM);
            currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
            currentMiddleLight = middleSensor.alpha();
            lastExitReadTime = tempoInicio;
            lastEntryReadTime = tempoInicio;
            lastMiddleReadTime = tempoInicio;
            return;
        }
        if (tempoInicio - lastExitReadTime >= EXIT_READ_INTERVAL_MS) {
            currentExitDist = exitSensor.getDistance(DistanceUnit.CM);
            lastExitReadTime = tempoInicio;
        } else if (tempoInicio - lastEntryReadTime >= ENTRY_READ_INTERVAL_MS) {
            currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
            lastEntryReadTime = tempoInicio;
        } else if (tempoInicio - lastMiddleReadTime >= MIDDLE_READ_INTERVAL_MS) {
            currentMiddleLight = middleSensor.alpha();
            lastMiddleReadTime = tempoInicio;
        }

        boolean exitActive = getExitSensor();
        boolean middleActive = getMiddleSensor();
        boolean entryActive = getEntrySensor();

        if (!isInitialized) {
            pieceCount = inferPieceCount(exitActive, middleActive, entryActive);
            lastInferred = pieceCount;
            previousExitActive = exitActive;
            isInitialized = true;
        } else if (isShooting) {
            if (previousExitActive && !exitActive
                    && tempoInicio - lastExitFallTime > EXIT_FALL_DEBOUNCE_MS) {
                if (pieceCount > 0) pieceCount--;
                lastExitFallTime = tempoInicio;
            }
        } else {
            boolean recentlyShooting = tempoInicio - shootingEndTime < POST_SHOOT_PROTECTION_MS;

            if (!recentlyShooting) {
                pieceCount = inferWithDebounce(exitActive, middleActive, entryActive);
            } else {
                int inferred = inferPieceCount(exitActive, middleActive, entryActive);
                if (inferred < pieceCount) {
                    if (inferred != lastFallInferred) {
                        lastFallInferred = inferred;
                        lastFallTime = tempoInicio;
                    }
                    if (tempoInicio - lastFallTime >= FALL_DEBOUNCE_MS) {
                        pieceCount = inferred;
                        lastInferred = inferred;
                    }
                }
            }
        }

        previousExitActive = exitActive;

        telemetry.addData("Indexer Pieces", pieceCount);
        telemetry.addData("Is Shooting?", isShooting);
        telemetry.addData("Exit Dist (CM)", currentExitDist);
        telemetry.addData("Exit Triggered", exitActive);
        telemetry.addData("Middle Light", currentMiddleLight);
        telemetry.addData("Middle Triggered", middleActive);
        telemetry.addData("Entry Dist (CM)", currentEntryDist);
        telemetry.addData("Entry Triggered", entryActive);

        long tempoFim = System.currentTimeMillis();
        telemetry.addData(">> Tempo Sensores Indexer (ms)", tempoFim - tempoInicio);
    }

    private int inferWithDebounce(boolean exit, boolean middle, boolean entry) {
        int inferred = inferPieceCount(exit, middle, entry);

        if (inferred > pieceCount) {
            if (inferred != lastInferred) {
                lastInferred = inferred;
                lastRiseTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - lastRiseTime >= RISE_DEBOUNCE_MS) {
                return inferred;
            }
            return pieceCount;
        } else if (inferred < pieceCount) {
            if (inferred != lastFallInferred) {
                lastFallInferred = inferred;
                lastFallTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis() - lastFallTime >= FALL_DEBOUNCE_MS) {
                lastInferred = inferred;
                return inferred;
            }
            return pieceCount;
        } else {
            lastInferred = inferred;
            lastFallInferred = inferred;
            return inferred;
        }
    }

    private int inferPieceCount(boolean exit, boolean middle, boolean entry) {
        if (exit && middle && entry)    return 3;
        if (exit && middle && !entry)   return 2;
        if (exit && !middle && !entry)  return 1;
        if (!exit && !middle && !entry) return 0;
        if (!exit && !middle && entry)  return 1;
        if (!exit && middle && !entry)  return 1;
        if (!exit && middle && entry)   return 2;
        if (exit && !middle && entry)   return 2;
        return 0;
    }

    public void setShootingState(boolean state) {
        if (!state && isShooting) {
            shootingEndTime = System.currentTimeMillis();
            lastFallInferred = -1;
        }
        this.isShooting = state;
    }

    public boolean getExitSensor() {
        return currentExitDist < IndexerConstants.EXIT_DISTANCE_CM;
    }

    public boolean getMiddleSensor() {
        return currentMiddleLight > IndexerConstants.MIDDLE_LIGHT_THRESHOLD;
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