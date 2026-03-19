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

    // O SEGREDO 1: 'volatile' obriga a memória a se atualizar instantaneamente entre os núcleos do processador
    private volatile double currentEntryDist = 23069.0;
    private volatile double currentExitDist = 23069.0;

    private boolean isInitialized = false;
    private boolean isShooting = false;

    // O SEGREDO 2: A Thread Secundária
    private Thread sensorThread;

    public IndexerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        exitSensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.EXIT_SENSOR_NAME);
        entrySensor = hardwareMap.get(DistanceSensor.class, IndexerConstants.ENTRY_SENSOR_NAME);

        // Dá a partida no processamento paralelo assim que o robô liga!
        iniciarThreadDeSensores();
    }

    private void iniciarThreadDeSensores() {
        sensorThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    // ISSO AGORA TRAVA O NÚCLEO SECUNDÁRIO, NÃO O ROBÔ!
                    // O Pedro Pathing continua voando na Main Thread enquanto o sensor pensa.
                    currentEntryDist = entrySensor.getDistance(DistanceUnit.CM);
                    currentExitDist = exitSensor.getDistance(DistanceUnit.CM);

                    // Pausa de 30ms para não congestionar os cabos I2C
                    Thread.sleep(30);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt(); // Encerra limpo
                } catch (Exception e) {
                    // Ignora erros caso o cabo balance
                }
            }
        });
        // Daemon = Garante que essa thread morra instantaneamente quando você apertar o STOP no celular
        sensorThread.setDaemon(true);
        sensorThread.start();
    }

    @Override
    public void periodic() {
        long tempoInicio = System.currentTimeMillis();

        // A MÁGICA: Não tem mais 'getDistance()' bloqueando o loop!
        // O código só lê o valor que a Thread já deixou pronto na memória. Custo real: 0.0 ms.

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