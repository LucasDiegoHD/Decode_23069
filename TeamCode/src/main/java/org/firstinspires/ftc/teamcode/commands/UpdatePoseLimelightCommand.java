package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import java.util.Optional;

/**
 * Comando para atualizar a pose do robô utilizando a Limelight.
 * Implementa uma medida de proteção: caso a visão falhe, assume uma pose de fallback.
 */
public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Pose fallbackPose;

    /**
     * @param drivetrain O subsistema de tração que contém o Follower do Pedro Pathing.
     * @param vision O subsistema de visão Limelight.
     * @param fallbackPose A pose (ex: endPose) que o robô deve assumir se a Limelight não vir nada.
     */
    public UpdatePoseLimelightCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Pose fallbackPose) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.fallbackPose = fallbackPose;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        Log.d("UpdatePose", "Iniciando atualização de pose via Limelight...");

        // Pega a pose atual estimada pela odometria (Pinpoint/Deadwheels)
        Pose currentPose = drivetrain.getFollower().getPose();

        // Tenta obter a pose global da Limelight usando o heading atual para o MegaTag2
        Optional<Pose> poseOptional = vision.getRobotPose(currentPose.getHeading());

        // PROTEÇÃO: Se a pose for null (Optional vazio), o robô assume a fallbackPose
        if (poseOptional.isEmpty()) {
            drivetrain.getFollower().setPose(fallbackPose);
            Log.w("UpdatePose", "Limelight não detectou tags. Pose resetada para o fallback (endpose).");
            return;
        }

        Pose llPose = poseOptional.get();

        // FILTRO DE SANIDADE: Calcula a distância entre a odometria e a visão
        double distInches = Math.hypot(llPose.getX() - currentPose.getX(), llPose.getY() - currentPose.getY());
        double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

        // Se a Limelight indicar um salto impossível (ex: reflexo), também usamos o fallback
        if (distInches > maxDeltaInches) {
            drivetrain.getFollower().setPose(fallbackPose);
            Log.d("UpdatePose", "LL detectada, mas rejeitada por salto excessivo (" + distInches + " in). Usando fallback.");
            return;
        }

        // FUSÃO DE DADOS: Combina a odometria com a visão usando os pesos das constantes
        Pose fusedPose = getFusedPose(currentPose, llPose);
        drivetrain.getFollower().setPose(fusedPose);

        Log.d("UpdatePose", "Pose atualizada com sucesso! LL=" + llPose + " | Fused=" + fusedPose);
    }

    /**
     * Realiza a fusão ponderada entre a pose da odometria e a pose da Limelight.
     */
    @NonNull
    private Pose getFusedPose(Pose currentPose, Pose llPose) {
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;
        double sum = wOdo + wLL;

        if (sum == 0) return currentPose;

        wOdo /= sum;
        wLL /= sum;

        // Funde as coordenadas X e Y, mantendo o heading da odometria (geralmente mais estável em tempo real)
        double fusedX = currentPose.getX() * wOdo + llPose.getX() * wLL;
        double fusedY = currentPose.getY() * wOdo + llPose.getY() * wLL;

        return new Pose(fusedX, fusedY, currentPose.getHeading());
    }

    @Override
    public boolean isFinished() {
        return true; // Comando instantâneo
    }
}