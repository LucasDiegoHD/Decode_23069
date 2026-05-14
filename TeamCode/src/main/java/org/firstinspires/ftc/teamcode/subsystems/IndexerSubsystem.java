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
    private final DistanceSensor middleSensor;

    private int pieceCount = 0;
    private boolean isShooting = false;
    private long shootingEndTime = 0;
    private static final long POST_SHOOT_PROTECTION_MS = 500;
    private static final long RISE_DEBOUNCE_MS = 200;

    private volatile double currentEntryDist = 23069.0;
    private volatile double currentExitDist = 23069.0;
    private volatile double currentMiddleDist = 23069.0;

    private boolean isInitialized = false;
    private long startTime = 0;
    private static final long INIT_DELAY_MS = 150;

    private int lastInferred = 0;
    private long lastRiseTime = 0;

    private Thread sensorThread;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);
        middleSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.MIDDLE_SENSOR_NAME);

        startTime = System.currentTimeMillis();
        iniciarThreadDeSensores();
    }

    private void iniciarThreadDeSensores() {
        sensorThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
                    currentExitDist = exitSensor.getDistance(DistanceUnit.CM);
                    currentMiddleDist = middleSensor.getDistance(DistanceUnit.CM);
                    Thread.sleep(40);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } catch (Exception e) {
                }
            }
        });
        sensorThread.setDaemon(true);
        sensorThread.start();
    }

    public void stopSensorThread() {
        if (sensorThread != null && sensorThread.isAlive()) {
            sensorThread.interrupt();
        }
    }

    @Override
    public void periodic() {
        long tempoInicio = System.currentTimeMillis();

        if (!isInitialized && System.currentTimeMillis() - startTime < INIT_DELAY_MS) {
            return;
        }

        boolean exitActive = getExitSensor();
        boolean middleActive = getMiddleSensor();
        boolean entryActive = getEntrySensor();

        if (!isInitialized) {
            pieceCount = inferPieceCount(exitActive, middleActive, entryActive);
            lastInferred = pieceCount;
            isInitialized = true;
        } else {
            boolean recentlyShooting = System.currentTimeMillis() - shootingEndTime < POST_SHOOT_PROTECTION_MS;

            if (!isShooting && !recentlyShooting) {
                pieceCount = inferWithDebounce(exitActive, middleActive, entryActive);
            } else {
                int inferred = inferPieceCount(exitActive, middleActive, entryActive);
                if (inferred < pieceCount) {
                    pieceCount = inferred;
                    lastInferred = inferred;
                }
            }
        }

        telemetry.addData("Indexer Pieces", pieceCount);
        telemetry.addData("Is Shooting?", isShooting);
        telemetry.addData("Exit Dist (CM)", currentExitDist);
        telemetry.addData("Exit Triggered", exitActive);
        telemetry.addData("Middle Dist (CM)", currentMiddleDist);
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
        } else {
            lastInferred = inferred;
            lastRiseTime = System.currentTimeMillis();
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
        if (!exit && middle && entry)   return 1;
        if (exit && !middle && entry)   return 2;
        return 0;
    }

    public void setShootingState(boolean state) {
        if (!state && isShooting) {
            shootingEndTime = System.currentTimeMillis();
        }
        this.isShooting = state;
    }

    public boolean getExitSensor() {
        return currentExitDist < IndexerConstants.EXIT_DISTANCE_CM;
    }

    public boolean getMiddleSensor() {
        return currentMiddleDist < IndexerConstants.MIDDLE_DISTANCE_CM;
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