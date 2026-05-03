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
    private boolean previousExitState = false;

    private long lastEntryEventTime = 0;
    private long lastExitEventTime = 0;

    private static final long DEBOUNCE_DELAY_MS = 100;

    private volatile double currentEntryDist = 23069.0;
    private volatile double currentExitDist = 23069.0;

    private boolean isInitialized = false;
    private long startTime = 0;
    private static final long INIT_DELAY_MS = 150;

    private boolean isShooting = false;
    private long shootingEndTime = 0;
    private static final long POST_SHOOT_PROTECTION_MS = 500;

    private Thread sensorThread;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);
        startTime = System.currentTimeMillis();
        iniciarThreadDeSensores();
    }

    private void iniciarThreadDeSensores() {
        sensorThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
                    currentExitDist = exitSensor.getDistance(DistanceUnit.CM);
                    Thread.sleep(50);
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

        boolean entryActive = getEntrySensor();
        boolean exitActive = getExitSensor();

        if (!isInitialized) {
            if (System.currentTimeMillis() - startTime < INIT_DELAY_MS) {
                return;
            }
            if (exitActive && entryActive) {
                pieceCount = IndexerConstants.MAX_PIECE_CAPACITY;
            } else if (exitActive || entryActive) {
                pieceCount = 1;
            } else {
                pieceCount = 0;
            }
            previousEntryState = entryActive;
            previousExitState = exitActive;
            isInitialized = true;
        }

        boolean recentlyShooting = System.currentTimeMillis() - shootingEndTime < POST_SHOOT_PROTECTION_MS;

        if (exitActive && entryActive && !isShooting) {
            pieceCount = IndexerConstants.MAX_PIECE_CAPACITY;
        }
        else if (exitActive && !entryActive && pieceCount < 1) {
            pieceCount = 1;
        }
        else if (entryActive && !exitActive && pieceCount < 1) {
            pieceCount = 1;
        }

        else if (!exitActive && !entryActive && !isShooting && !recentlyShooting) {
            pieceCount = 0;
        }


        if (entryActive && !previousEntryState
                && System.currentTimeMillis() - lastEntryEventTime > DEBOUNCE_DELAY_MS) {
            if (pieceCount < IndexerConstants.MAX_PIECE_CAPACITY) {
                pieceCount++;
            }
            lastEntryEventTime = System.currentTimeMillis();
        }

        if (!exitActive && previousExitState && isShooting
                && System.currentTimeMillis() - lastExitEventTime > DEBOUNCE_DELAY_MS) {
            if (pieceCount > 0) {
                pieceCount--;
            }
            lastExitEventTime = System.currentTimeMillis();
        }

        previousEntryState = entryActive;
        previousExitState = exitActive;

        telemetry.addData("Indexer Pieces", pieceCount);
        telemetry.addData("Is Shooting?", isShooting);
        telemetry.addData("Entry Dist (CM)", currentEntryDist);
        telemetry.addData("Entry Triggered", entryActive);
        telemetry.addData("Exit Dist (CM)", currentExitDist);
        telemetry.addData("Exit Triggered", exitActive);

        long tempoFim = System.currentTimeMillis();
        telemetry.addData(">> Tempo Sensores Indexer (ms)", tempoFim - tempoInicio);
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