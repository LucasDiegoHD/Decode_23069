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
    private volatile double currentEntryDist = 23069.0;
    private volatile double currentExitDist = 23069.0;

    private boolean isInitialized = false;
    private boolean isShooting = false;

    private Thread sensorThread;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);

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
                    // Sensor falhou — mantém último valor conhecido
                }
            }
        });
        sensorThread.setDaemon(true);
        sensorThread.start();
    }

    /**
     * Para a thread de leitura dos sensores.
     * Deve ser chamado no end() do OpMode para evitar duas threads rodando
     * simultaneamente se o OpMode for reiniciado sem matar o processo.
     */
    public void stopSensorThread() {
        if (sensorThread != null && sensorThread.isAlive()) {
            sensorThread.interrupt();
        }
    }

    @Override
    public void periodic() {
        long tempoInicio = System.currentTimeMillis();

        boolean currentEntryState = getEntrySensor();
        boolean currentExitState = getExitSensor();

        if (!isInitialized) {
            // CORRIGIDO: era if/if/else, o que fazia o segundo if sobrescrever
            // o primeiro quando ambos os sensores estavam ativos.
            // Agora é if/else if/else — cada caso é exclusivo.
            if (currentExitState && currentEntryState) {
                pieceCount = IndexerConstants.MAX_PIECE_CAPACITY;
            } else if (currentExitState || currentEntryState) {
                pieceCount = 1;
            } else {
                pieceCount = 0;
            }
            previousEntryState = currentEntryState;
            previousExitState = currentExitState;
            isInitialized = true;
        }

        if (currentEntryState && !previousEntryState
                && (System.currentTimeMillis() - lastEntryCountTime > DEBOUNCE_DELAY_MS)) {
            if (pieceCount < IndexerConstants.MAX_PIECE_CAPACITY) {
                pieceCount++;
            }
            lastEntryCountTime = System.currentTimeMillis();
        }

        if (!currentExitState && previousExitState && isShooting
                && (System.currentTimeMillis() - lastExitCountTime > DEBOUNCE_DELAY_MS)) {
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

        long tempoFim = System.currentTimeMillis();
        telemetry.addData(">> Tempo Sensores Indexer (ms)", tempoFim - tempoInicio);
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