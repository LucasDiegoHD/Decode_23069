package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command to spin the shooter motors to a specific velocity based on distance AND chassis momentum.
 */
public class AdjustShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final double targetX;
    private final double targetY;

    // Calibração: Quantos RPM compensar para cada polegada/s de velocidade de aproximação.
    // Exemplo: Se o robô aproxima a 10 in/s, diminui 100 RPM. (Ajuste na quadra!)
    private static final double RPM_PER_INCH_PER_SEC = 3.0;

    // LIMITES DE SEGURANÇA (Ajuste de acordo com o seu motor. Ex: goBILDA 1:1 é ~6000 RPM limite)
    private static final double MAX_SAFE_RPM = 4500;
    private static final double MIN_SAFE_RPM = 1000.0;

    public AdjustShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, DrivetrainSubsystem drivetrain, double targetX, double targetY) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    @Override
    public void initialize() {
        double distance = vision.getDirectDistanceToTarget().orElse(0.0);

        Pose pose = drivetrain.getFollower().getPose();
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double groundDistance = Math.hypot(dx, dy);

        // FALLBACK ODÔMETRO (Corrigido)
        // Se a câmera piscar ou não ver (distance <= 0), usamos o Pedro Pathing imediatamente.
        if (distance <= 0.1) {
            double deltaZ = 38.75;
            double distanceInches = Math.hypot(groundDistance, deltaZ);
            distance = distanceInches / 39.3701; // Converte para metros para o polinômio
        }

        // 1. POLINÔMIO: Calcula o RPM Base
        double baseRpm = ShooterConstants.RPM_N0 + ShooterConstants.RPM_N1 * distance + ShooterConstants.RPM_N2 * Math.pow(distance, 2);

        // 2. COMPENSAÇÃO DE MOMENTO (Efeito Doppler do RPM)
        Vector velocity = drivetrain.getFollower().getVelocity();
        double velTowardsGoal = 0.0;

        // Evita divisão por zero se estiver no centro exato do alvo
        if (groundDistance > 1.0) {
            double dirX = dx / groundDistance;
            double dirY = dy / groundDistance;
            velTowardsGoal = (velocity.getXComponent() * dirX) + (velocity.getYComponent() * dirY);
        }

        // Se vamos NA DIREÇÃO do cesto (positivo), subtraímos RPM porque a bola já tem embalo.
        // Se estamos DANDO RÉ (negativo), a matemática vai somar (- com - dá +).
        double finalRpm = baseRpm - (velTowardsGoal * RPM_PER_INCH_PER_SEC);

        // 3. CAMADAS DE SEGURANÇA CRÍTICAS
        if (distance > VisionConstants.LONGEST_DISTANCE) {
            finalRpm = VisionConstants.LONGEST_RPM;
        }

        // A BARREIRA DE HARDWARE: Impede que o código mande RPM negativo (robô puxar a bola)
        // ou force o motor além do limite físico, o que causaria sobrecarga e "Stall".
        finalRpm = Math.max(MIN_SAFE_RPM, Math.min(finalRpm, MAX_SAFE_RPM));

        shooter.setTargetVelocity(finalRpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}