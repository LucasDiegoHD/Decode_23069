package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyConstants; // Seus arquivos de constantes
import org.firstinspires.ftc.teamcode.subsystems.HuskySubsystem;

import java.util.Optional;

public class ChaseArtifactCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final HuskySubsystem vision;
    private final PIDController turnPID, drivePID;

    public ChaseArtifactCommand(DrivetrainSubsystem drivetrain, HuskySubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // PIDs
        this.turnPID = new PIDController(HuskyConstants.TURN_KP, HuskyConstants.TURN_KI, HuskyConstants.TURN_KD);
        this.drivePID = new PIDController(HuskyConstants.DRIVE_KP, HuskyConstants.DRIVE_KI, HuskyConstants.DRIVE_KD);

        this.turnPID.setSetPoint(HuskyConstants.CENTER_X_PIXELS); // 160
        this.drivePID.setSetPoint(HuskyConstants.TARGET_DISTANCE_INCHES); // 1.0

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        turnPID.reset();
        drivePID.reset();
    }

    private long timeSinceLostTarget = 0;
    private boolean wasClose = false;

    @Override
    public void execute() {
        Optional<HuskyLens.Block> target = vision.getClosestAnyArtifact();

        double drivePower = 0;
        double turnPower = 0;

        if (target.isPresent()) {
            HuskyLens.Block block = target.get();
            double currentDist = vision.getDistanceToBlock(block);

            // Reseta o timer de perda de alvo
            timeSinceLostTarget = System.currentTimeMillis();

            // --- LÓGICA DE DECISÃO ---

            if (currentDist > HuskyConstants.TARGET_DISTANCE_INCHES) {
                // ZONA 1: APROXIMAÇÃO (Longe)
                // Usa PID para chegar perto suavemente
                wasClose = false;
                turnPower = turnPID.calculate(block.x);
                drivePower = -drivePID.calculate(currentDist); // Negativo para ir para frente

                // Limita a velocidade para não ir rápido demais
                drivePower = MathUtils.clamp(drivePower, -0.6, 0.6);

            } else {
                // ZONA 2: COLETA (Perto - Menos de 8 polegadas / 20cm)
                // AQUI ESTÁ O SEGREDO: Ignora o PID de distância!
                // Força o robô a ir para cima do artefato.
                wasClose = true;

                drivePower = -0.4; // Velocidade fixa para frente (ajuste conforme seu robô)
                turnPower = turnPID.calculate(block.x) * 0.5; // Alinha um pouco, mas com menos força

                // IMPORTANTE: Se tiver comando de Intake, ligue aqui!
                // intake.setPower(1.0);
            }

            drivetrain.driveRobotCentric(drivePower, 0, turnPower);

        } else {
            // --- O ROBÔ NÃO ESTÁ VENDO NADA ---

            // Se ele estava perto (wasClose) e perdeu a visão faz pouco tempo (menos de 1 seg),
            // significa que o bloco está DEBAIXO do intake (Ponto Cego).
            if (wasClose && (System.currentTimeMillis() - timeSinceLostTarget < 1000)) {

                // CONTINUA ANDANDO PARA FRENTE "NO ESCURO"
                drivetrain.driveRobotCentric(-0.4, 0, 0);
                // intake.setPower(1.0);

            } else {
                // Realmente perdeu ou já coletou
                drivetrain.driveRobotCentric(0, 0, 0);
                // intake.setPower(0);
                wasClose = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Retorna false para rodar "enquanto o botão estiver apertado"
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Garante que o robô pare quando você soltar o botão
        drivetrain.driveRobotCentric(0, 0, 0);
    }
}