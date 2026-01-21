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

    public UpdatePoseLimelightCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Pose fallbackPose) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.fallbackPose = fallbackPose;
        addRequirements(vision);
    }

    public static void forceHardReset(DrivetrainSubsystem drive, VisionSubsystem vis, double targetHeadingDegrees) {
        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);
        Pose currentPose = drive.getFollower().getPose();

        drive.getFollower().setPose(new Pose(currentPose.getX(), currentPose.getY(), targetHeadingRad));


        vis.getRobotPoseMT2(targetHeadingRad).ifPresent(mt2Pose -> {

            drive.getFollower().setPose(new Pose(
                    mt2Pose.getX(),
                    mt2Pose.getY(),
                    targetHeadingRad
            ));
            Log.i("Vision", "HARD RESET: Posição atualizada 100% via Limelight");
        });
    }

    @Override
    public void initialize() {
        Optional<Pose> mt1Pose = vision.getRobotPoseMT1();

        // 1. TRATAMENTO DE POSE VAZIA
        if (mt1Pose.isEmpty()) {
            if (!hasInitialized) {
                drivetrain.getFollower().setPose(fallbackPose);
                Log.w("Vision", "Pose vazia e não inicializado. Fallback aplicado.");
            } else {
                Log.d("Vision", "Pose vazia. Mantendo odometria pura.");
            }
            return;
        }

        Pose llPoseMT1 = mt1Pose.get();
        Pose currentPose = drivetrain.getFollower().getPose();

        if (!hasInitialized) {
            drivetrain.getFollower().setPose(llPoseMT1);
            hasInitialized = true;
            Log.d("Vision", "MT1 Seed/Re-Sync Sucesso (Ângulo Corrigido): " + llPoseMT1);
            return;
        }

        double currentYaw = currentPose.getHeading();
        vision.getRobotPoseMT2(currentYaw).ifPresent(llPoseMT2 -> {

            double distInches = Math.hypot(llPoseMT2.getX() - currentPose.getX(), llPoseMT2.getY() - currentPose.getY());
            double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

            if (distInches < maxDeltaInches) {
                Pose fusedPose = getFusedPose(currentPose, llPoseMT2);
                drivetrain.getFollower().setPose(fusedPose);
            } else {
                Log.w("Vision", "MT2 rejeitado: Salto de " + distInches + " in (Reflexo provável).");
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
    }
}