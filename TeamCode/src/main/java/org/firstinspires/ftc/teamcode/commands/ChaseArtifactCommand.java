package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.templates.HuskyConstants.BLIND_DURATION_MS;
import static org.firstinspires.ftc.teamcode.subsystems.templates.HuskyConstants.MIN_DRIVE_SPEED_PID;
import static org.firstinspires.ftc.teamcode.subsystems.templates.HuskyConstants.TURBO_SPEED;
import static org.firstinspires.ftc.teamcode.subsystems.templates.HuskyConstants.TURBO_THRESHOLD;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.HuskyConstants;
import org.firstinspires.ftc.teamcode.subsystems.templates.HuskySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Optional;

public class ChaseArtifactCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final HuskySubsystem vision;
    private final PIDController turnPID, drivePID;

    private final IntakeSubsystem intakeSubsystem;

    private static final double MAX_TURN_SPEED = 0.25;

    private static final double BLIND_SPEED = -0.4;

    private long timeSinceLostTarget = 0;
    private boolean wasClose = false;

    public ChaseArtifactCommand(DrivetrainSubsystem drivetrain, HuskySubsystem vision, IntakeSubsystem intakeSubsystem) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.intakeSubsystem = intakeSubsystem;

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
    }

    @Override
    public void execute() {
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
                // Ignora PID. Vai com tudo para chegar rápido.
                wasClose = false;
                drivePower = TURBO_SPEED; // Ex: -0.85

            } else if (currentDist > HuskyConstants.TARGET_DISTANCE_INCHES) {
                // --- ZONA 2: APROXIMAÇÃO FINA (Entre 12 e 5 pol) ---
                // Usa PID para desacelerar suavemente e não bater com tudo
                wasClose = false;

                intakeSubsystem.run();

                drivePower = drivePID.calculate(currentDist);

                // Garante que não fique lento demais (Feedforward)
                if (Math.abs(drivePower) > 0.01 && Math.abs(drivePower) < MIN_DRIVE_SPEED_PID) {
                    drivePower = Math.signum(drivePower) * MIN_DRIVE_SPEED_PID;
                }

                // Limita para não ser mais rápido que o turbo
                drivePower = MathUtils.clamp(drivePower, TURBO_SPEED, -MIN_DRIVE_SPEED_PID);

            } else {
                // --- ZONA 3: ATAQUE (Perto < 5 pol) ---
                wasClose = true;
                drivePower = BLIND_SPEED; // Ex: -0.6 constante

                // Reduz giro para estabilizar a coleta
                turnPower = turnPower * 0.1;
            }

            // Envia comando (Strafe=0, Drive, Turn)
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
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotCentric(0, 0, 0);
    }
}