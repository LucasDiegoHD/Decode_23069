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
 * Competitive Pose Update Command - DECODE Edition.
 * Lógica: Reseeding via MT1 em cada re-aquisição de tag e tracking via MT2.
 */
public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Pose fallbackPose;

    private static boolean hasInitialized = false;
    private static boolean tagFoundLastCycle = false; // Memória de estado para forçar MT1

    public UpdatePoseLimelightCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Pose fallbackPose) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.fallbackPose = fallbackPose;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        Optional<Pose> mt1Pose = vision.getRobotPoseMT1();

        // 1. TRATAMENTO DE POSE VAZIA (SEGURANÇA SOLICITADA)
        if (mt1Pose.isEmpty()) {
            if (!hasInitialized) {
                // Nunca sincronizou e não vê tag: Usa fallback, mas mantêm false
                drivetrain.getFollower().setPose(fallbackPose);
                Log.w("Vision", "Pose vazia e não inicializado. Fallback aplicado.");
            } else {
                // Já sincronizou mas perdeu a tag: Confia na odometria (fused)
                Log.d("Vision", "Pose vazia. Mantendo odometria pura.");
            }
            tagFoundLastCycle = false; // Marcamos que perdemos a tag para forçar MT1 na volta
            return;
        }

        // 2. TRATAMENTO DE TAG VISÍVEL
        Pose llPoseMT1 = mt1Pose.get();
        Pose currentPose = drivetrain.getFollower().getPose();

        // FASE 1: SEEDING / RE-SYNC (MegaTag 1)
        // Se nunca iniciou OU se acabamos de reencontrar a tag após perdê-la
        if (!hasInitialized ||!tagFoundLastCycle) {
            drivetrain.getFollower().setPose(llPoseMT1);
            hasInitialized = true;
            tagFoundLastCycle = true;
            Log.d("Vision", "MT1 Seed/Re-Sync Sucesso (Ângulo Corrigido): " + llPoseMT1);
            return;
        }

        // FASE 2: TRACKING (MegaTag 2)
        // Agora que o ângulo está garantido pelo MT1 anterior, usamos MT2 para estabilidade
        double currentYaw = currentPose.getHeading();
        vision.getRobotPoseMT2(currentYaw).ifPresent(llPoseMT2 -> {

            double distInches = Math.hypot(llPoseMT2.getX() - currentPose.getX(), llPoseMT2.getY() - currentPose.getY());
            double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

            // Filtro de Rejeição Industrial contra reflexos e saltos [2]
            if (distInches < maxDeltaInches) {
                Pose fusedPose = getFusedPose(currentPose, llPoseMT2);
                drivetrain.getFollower().setPose(fusedPose);
                tagFoundLastCycle = true;
            } else {
                Log.w("Vision", "MT2 rejeitado: Salto de " + distInches + " in (Reflexo provável).");
                // Não alteramos tagFoundLastCycle para tentar novamente no próximo loop
            }
        });
    }

    /**
     * Fusão ponderada conforme padrão de elite.
     * Mantém o heading da odometria no tracking para evitar glitches de 360/0. [8, 2]
     */
    @NonNull
    private Pose getFusedPose(Pose currentPose, Pose llPose) {
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;
        double sum = wOdo + wLL;
        if (sum == 0) return currentPose;

        wOdo /= sum;
        wLL /= sum;

        return new Pose(
                currentPose.getX() * wOdo + llPose.getX() * wLL,
                currentPose.getY() * wOdo + llPose.getY() * wLL,
                currentPose.getHeading()
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public static void resetLocalizationStatus() {
        hasInitialized = false;
        tagFoundLastCycle = false;
    }
}