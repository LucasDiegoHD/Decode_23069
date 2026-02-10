package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AdjustShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;

    // Coordenadas do alvo recebidas por parâmetro
    private final double targetX;
    private final double targetY;

    public AdjustShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, DrivetrainSubsystem drivetrain, double targetX, double targetY) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;

        // Não requeremos o drivetrain para não travar a movimentação enquanto calcula
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        double distance = 0;

        // 1. Tenta pegar a distância pela Câmera (Prioridade: Visão Real)
        // O método orElse(-1.0) ajuda a saber se falhou retornando um valor impossível
        double visionDist = vision.getDirectDistanceToTarget().orElse(-1.0);

        if (visionDist > 0) {
            distance = visionDist;
        } else {
            // 2. BACKUP: Se a câmera falhou, calcula via Odometria (Pedro Pathing)
            Pose robotPose = drivetrain.getFollower().getPose();

            // Distância Euclidiana: √((x2-x1)² + (y2-y1)²)
            distance = Math.hypot(targetX - robotPose.getX(), targetY - robotPose.getY());
        }

        // 3. Calcula RPM baseado na distância encontrada
        double rpm;

        if (distance > VisionConstants.LONGEST_DISTANCE) {
            rpm = VisionConstants.LONGEST_RPM;
        } else {
            // Polinômio de regressão (Curva de calibração)
            rpm = ShooterConstants.RPM_N0
                    + ShooterConstants.RPM_N1 * distance
                    + ShooterConstants.RPM_N2 * Math.pow(distance, 2);
        }

        // Segurança: Se a distância for zero ou negativa (erro grave), usa tiro curto padrão
        if (distance <= 0.1) {
            rpm = ShooterConstants.TARGET_VELOCITY_SHORT;
        }

        shooter.setTargetVelocity(rpm);
    }

    @Override
    public boolean isFinished() {
        return true; // Comando instantâneo
    }
}