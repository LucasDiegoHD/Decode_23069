package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * Multi-Sensor Serial Indexer Subsystem Template
 *
 * Overview:
 * Monitors and manages game piece count and queue flow inside the robot's serial conveyor
 * using a 3-sensor array (Entry Distance, Middle Color/Light Sensor, and Exit Distance).
 *
 * Hardware Configuration:
 * - Entry Distance Sensor: "sensorEntry" (I2C Bus)
 * - Middle Color Sensor:   "middleSensor" (I2C Bus)
 * - Exit Distance Sensor:  "sensor_color_distance" (I2C Bus)
 *
 * Performance and State Tracking Strategy:
 * - I2C Time-Slicing (Staggered Polling): Polling three I2C sensors in a single loop
 *   tick introduces significant bus latency (~30-60 ms). This subsystem rotates sensor reads
 *   across successive loop frames with 150 ms intervals.
 * - State Inference Matrix: Maps the 8 possible combinations of 3 binary sensor states
 *   to physical piece counts (0, 1, 2, or 3).
 * - Asymmetric Debounce Filtering: Applies separate rise (200 ms) and fall (80 ms)
 *   debounce filters to avoid false count transitions caused by piece vibration.
 * - Shooting State Blanking: Decrements count on the falling edge of the exit sensor
 *   and protects against intake re-inference during the 500 ms post-fire window.
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
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

    /**
     * Constructs a new IndexerSubsystem and acquires sensor hardware handles.
     *
     * @param hardwareMap Robot hardware map for device retrieval.
     * @param telemetry   Telemetry manager for diagnostic logging.
     */
    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);
        middleSensor = hardwareMap.get(RevColorSensorV3.class, IndexerConstants.MIDDLE_SENSOR_NAME);

        startTime = System.currentTimeMillis();
    }

    /**
     * Periodic sensor polling and piece count estimation loop.
     * Executes staggered I2C sensor reads, runs the debounce filtering machine, and publishes debug telemetry.
     */
    @Override
    public void periodic() {
        long tempoInicio = System.currentTimeMillis();

        // Initial warm-up delay before enabling debounce transitions
        if (!isInitialized && tempoInicio - startTime < INIT_DELAY_MS) {
            currentExitDist = exitSensor.getDistance(DistanceUnit.CM);
            currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
            currentMiddleLight = middleSensor.alpha();
            lastExitReadTime = tempoInicio;
            lastEntryReadTime = tempoInicio;
            lastMiddleReadTime = tempoInicio;
            return;
        }

        // Staggered I2C sensor polling (Time-Slicing)
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
            // During active firing: decrement on the falling edge of the exit sensor
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
        if (DataStorage.DEBUG_MODE) {
            telemetry.addData("Is Shooting?", isShooting);
            telemetry.addData("Exit Dist (CM)", currentExitDist);
            telemetry.addData("Exit Triggered", exitActive);
            telemetry.addData("Middle Light", currentMiddleLight);
            telemetry.addData("Middle Triggered", middleActive);
            telemetry.addData("Entry Dist (CM)", currentEntryDist);
            telemetry.addData("Entry Triggered", entryActive);
        }

        long tempoFim = System.currentTimeMillis();
        telemetry.addData(">> Indexer Sensor Time (ms)", tempoFim - tempoInicio);
    }

    /**
     * Applies rise and fall time-delay debouncing to inferred piece counts.
     *
     * @param exit   Exit sensor active state.
     * @param middle Middle light sensor active state.
     * @param entry  Entry sensor active state.
     * @return Debounced piece count.
     */
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

    /**
     * Maps the 3-sensor boolean trigger states to the most probable physical piece count (0 to 3).
     *
     * @param exit   True if exit sensor is obstructed.
     * @param middle True if middle sensor is obstructed.
     * @param entry  True if entry sensor is obstructed.
     * @return Physical piece count estimate.
     */
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

    /**
     * Sets whether the robot is actively executing a firing sequence.
     * When active, indexer switches to tracking falling edges on the exit sensor.
     *
     * @param state True when shooting is in progress; false otherwise.
     */
    public void setShootingState(boolean state) {
        if (!state && isShooting) {
            shootingEndTime = System.currentTimeMillis();
            lastFallInferred = -1;
        }
        this.isShooting = state;
    }

    /**
     * Returns whether the exit distance sensor is currently obstructed by a piece.
     *
     * @return True if distance is within EXIT_DISTANCE_CM.
     */
    public boolean getExitSensor() {
        return currentExitDist < IndexerConstants.EXIT_DISTANCE_CM;
    }

    /**
     * Returns whether the middle color/light sensor detects a piece in the middle slot.
     *
     * @return True if measured alpha exceeds MIDDLE_LIGHT_THRESHOLD.
     */
    public boolean getMiddleSensor() {
        return currentMiddleLight > IndexerConstants.MIDDLE_LIGHT_THRESHOLD;
    }

    /**
     * Returns whether the entry distance sensor detects a piece entering the indexer.
     *
     * @return True if distance is within ENTRY_DISTANCE_CM.
     */
    public boolean getEntrySensor() {
        return currentEntryDist < IndexerConstants.ENTRY_DISTANCE_CM;
    }

    /**
     * Returns the current estimated piece count inside the indexer (0 to 3).
     *
     * @return Piece count.
     */
    public int getPieceCount() {
        return pieceCount;
    }

    /**
     * Returns whether the indexer contains at least one piece.
     *
     * @return True if pieceCount > 0.
     */
    public boolean hasPieces() {
        return pieceCount > 0;
    }

    /**
     * Returns whether the indexer has reached its maximum capacity.
     *
     * @return True if pieceCount >= MAX_PIECE_CAPACITY.
     */
    public boolean isFull() {
        return pieceCount >= IndexerConstants.MAX_PIECE_CAPACITY;
    }

    /**
     * Manually overrides the internal piece count tracker.
     *
     * @param count Desired piece count (clamped to [0, MAX_PIECE_CAPACITY]).
     */
    public void setPieceCount(int count) {
        if (count < 0) count = 0;
        if (count > IndexerConstants.MAX_PIECE_CAPACITY) count = IndexerConstants.MAX_PIECE_CAPACITY;
        this.pieceCount = count;
    }
}