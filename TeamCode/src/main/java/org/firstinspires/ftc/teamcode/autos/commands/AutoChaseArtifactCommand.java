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
import org.firstinspires.ftc.teamcode.subsystems.IndexerConstants;

import java.util.Optional;

public class AutoChaseArtifactCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final HuskySubsystem vision;
    private final PIDController turnPID, drivePID;
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexer;

    private static final double MAX_TURN_SPEED = 0.25;
    private static final double BLIND_SPEED = -0.4;

    private long timeSinceLostTarget = 0;
    private boolean wasClose = false;

    // Variáveis da Lógica de Finalização
    private long startTime = 0;
    private static final long TIMEOUT_MS = 3000; // Tempo máximo caçando (3 segundos)

    public AutoChaseArtifactCommand(DrivetrainSubsystem drivetrain, HuskySubsystem vision, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.intakeSubsystem = intakeSubsystem;
        this.indexer = indexer;

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
        timeSinceLostTarget = 0;

        // Inicia o cronômetro
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // === LÓGICA ORIGINAL EXATAMENTE COMO VOCÊ ESCREVEU ===
        Optional<HuskyLens.Block> target = vision.getClosestAnyArtifact();

        double drivePower = 0;
        double turnPower = 0;

        if (target.isPresent()) {
            HuskyLens.Block block = target.get();
            double currentDist = vision.getDistanceToBlock(block);

            // Reseta timer
            timeSinceLostTarget = System.currentTimeMillis();

            // === 1. GIRO (Suave e Controlado) ===
            double errorX = block.x - HuskyConstants.CENTER_X_PIXELS;

            if (Math.abs(errorX) > HuskyConstants.DEADZONE_ALIGN_PIXELS) {
                turnPower = turnPID.calculate(block.x);
                turnPower = MathUtils.clamp(turnPower, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            } else {
                turnPower = 0;
            }

            // === 2. DISTÂNCIA (LÓGICA DE 3 ZONAS) ===

            if (currentDist > TURBO_THRESHOLD) {
                // --- ZONA 1: TURBO (Muito Longe > 12 pol) ---
                wasClose = false;
                drivePower = TURBO_SPEED;

            } else if (currentDist > HuskyConstants.TARGET_DISTANCE_INCHES) {
                // --- ZONA 2: APROXIMAÇÃO FINA (Entre 12 e 5 pol) ---
                wasClose = false;

                intakeSubsystem.run(); // O seu intake original ligava aqui

                drivePower = drivePID.calculate(currentDist);

                if (Math.abs(drivePower) > 0.01 && Math.abs(drivePower) < MIN_DRIVE_SPEED_PID) {
                    drivePower = Math.signum(drivePower) * MIN_DRIVE_SPEED_PID;
                }

                drivePower = MathUtils.clamp(drivePower, TURBO_SPEED, -MIN_DRIVE_SPEED_PID);

            } else {
                // --- ZONA 3: ATAQUE (Perto < 5 pol) ---
                wasClose = true;
                drivePower = BLIND_SPEED;

                turnPower = turnPower * 0.1;
            }

            drivetrain.driveRobotCentric(0, drivePower, turnPower);

        } else {
            // === MODO CEGO ===
            if (wasClose && (System.currentTimeMillis() - timeSinceLostTarget < BLIND_DURATION_MS)) {
                drivetrain.driveRobotCentric(0, BLIND_SPEED, 0);
            } else {
                drivetrain.driveRobotCentric(0, 0, 0);
                wasClose = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        // CONDIÇÃO 1: Acabou o tempo (Evita que o robô fique preso caçando para sempre)
        if (System.currentTimeMillis() - startTime > TIMEOUT_MS) {
            return true;
        }

        // CONDIÇÃO 2: O robô engoliu 3 bolas
        if (indexer.getPieceCount() >= IndexerConstants.MAX_PIECE_CAPACITY) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotCentric(0, 0, 0);
        intakeSubsystem.stop();
    }
}