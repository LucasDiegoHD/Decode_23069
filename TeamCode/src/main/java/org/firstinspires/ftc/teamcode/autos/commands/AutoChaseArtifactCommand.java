package org.firstinspires.ftc.teamcode.autos.commands;

import static org.firstinspires.ftc.teamcode.subsystems.HuskyConstants.BLIND_DURATION_MS;
import static org.firstinspires.ftc.teamcode.subsystems.HuskyConstants.MIN_DRIVE_SPEED_PID;
import static org.firstinspires.ftc.teamcode.subsystems.HuskyConstants.TURBO_SPEED;
import static org.firstinspires.ftc.teamcode.subsystems.HuskyConstants.TURBO_THRESHOLD;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyConstants;
import org.firstinspires.ftc.teamcode.subsystems.HuskySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

import java.util.Optional;

public class AutoChaseArtifactCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final HuskySubsystem vision;
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexer;
    private final PIDController turnPID, drivePID;

    private final int maxBalls;
    private int ballsCollected = 0;

    private boolean previousEntryState = false;

    // --- VARIÁVEIS DE PROTEÇÃO (DEBOUNCE) ---
    // Tempo que o sensor fica "cego" após contar uma bola para evitar contagem dupla (Tremor)
    private static final long DEBOUNCE_DELAY_MS = 300;
    private long lastBallCountTime = 0;

    private static final double MAX_TURN_SPEED = 0.25;
    private static final double BLIND_SPEED = -0.4;

    private long timeSinceLostTarget = 0;
    private long startSearchTime = 0;
    private boolean wasClose = false;
    private boolean isDone = false;

    private boolean everSawBall = false;
    private static final long SEARCH_TIMEOUT_MS = 1500;

    public AutoChaseArtifactCommand(DrivetrainSubsystem drivetrain, HuskySubsystem vision, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer, int maxBalls) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.intakeSubsystem = intakeSubsystem;
        this.indexer = indexer;
        this.maxBalls = maxBalls;

        this.turnPID = new PIDController(HuskyConstants.TURN_KP, HuskyConstants.TURN_KI, HuskyConstants.TURN_KD);
        this.drivePID = new PIDController(HuskyConstants.DRIVE_KP, HuskyConstants.DRIVE_KI, HuskyConstants.DRIVE_KD);

        this.turnPID.setSetPoint(HuskyConstants.CENTER_X_PIXELS);
        this.drivePID.setSetPoint(HuskyConstants.TARGET_DISTANCE_INCHES);

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        turnPID.reset();
        drivePID.reset();
        wasClose = false;
        isDone = false;
        everSawBall = false;
        ballsCollected = 0;

        previousEntryState = indexer.getEntrySensor();
        lastBallCountTime = 0; // Zera o relógio de proteção

        timeSinceLostTarget = 0;
        startSearchTime = System.currentTimeMillis();

        intakeSubsystem.run();
    }

    @Override
    public void execute() {
        if (isDone) return;

        // =========================================================
        // 1. DETEÇÃO DE BORDA + PROTEÇÃO ANTI-TREMOR (DEBOUNCE)
        // =========================================================
        boolean currentEntryState = indexer.getEntrySensor();

        // "Se não via nada (!previous) e começou a ver (current) E já passou o tempo de proteção"
        if (currentEntryState && !previousEntryState && (System.currentTimeMillis() - lastBallCountTime > DEBOUNCE_DELAY_MS)) {
            ballsCollected++;
            lastBallCountTime = System.currentTimeMillis(); // Ativa o escudo de proteção temporal
            startSearchTime = System.currentTimeMillis(); // Renova o tempo de busca da câmara

            indexer.setPieceCount(ballsCollected);
        }

        previousEntryState = currentEntryState;

        // Se apanhou a quantidade que pedimos, finaliza o comando na hora!
        if (ballsCollected >= maxBalls) {
            drivetrain.driveRobotCentric(0, 0, 0);
            isDone = true;
            return;
        }

        // =========================================================
        // 2. LÓGICA DE CAÇA DA CÂMARA (HUNTER MODE)
        // =========================================================
        Optional<HuskyLens.Block> target = vision.getClosestAnyArtifact();
        double drivePower = 0;
        double turnPower = 0;

        if (target.isPresent()) {
            everSawBall = true;

            HuskyLens.Block block = target.get();
            double currentDist = vision.getDistanceToBlock(block);
            timeSinceLostTarget = System.currentTimeMillis();

            double errorX = block.x - HuskyConstants.CENTER_X_PIXELS;
            if (Math.abs(errorX) > HuskyConstants.DEADZONE_ALIGN_PIXELS) {
                turnPower = turnPID.calculate(block.x);
                turnPower = MathUtils.clamp(turnPower, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            }

            if (currentDist > TURBO_THRESHOLD) {
                wasClose = false;
                drivePower = TURBO_SPEED;
            } else if (currentDist > HuskyConstants.TARGET_DISTANCE_INCHES) {
                wasClose = false;
                drivePower = drivePID.calculate(currentDist);
                if (Math.abs(drivePower) > 0.01 && Math.abs(drivePower) < MIN_DRIVE_SPEED_PID) {
                    drivePower = Math.signum(drivePower) * MIN_DRIVE_SPEED_PID;
                }
                drivePower = MathUtils.clamp(drivePower, TURBO_SPEED, -MIN_DRIVE_SPEED_PID);
            } else {
                wasClose = true;
                drivePower = BLIND_SPEED;
                turnPower = turnPower * 0.1;
            }
            drivetrain.driveRobotCentric(0, drivePower, turnPower);

        } else {
            if (wasClose) {
                if (System.currentTimeMillis() - timeSinceLostTarget < BLIND_DURATION_MS) {
                    drivetrain.driveRobotCentric(0, BLIND_SPEED, 0);
                } else {
                    wasClose = false;
                    startSearchTime = System.currentTimeMillis();
                }
            } else {
                drivetrain.driveRobotCentric(0, 0, 0);

                if (System.currentTimeMillis() - startSearchTime > SEARCH_TIMEOUT_MS) {
                    isDone = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotCentric(0, 0, 0);
        intakeSubsystem.stop();
    }

    public boolean needsFallbackRoute() {
        return !everSawBall;
    }
}