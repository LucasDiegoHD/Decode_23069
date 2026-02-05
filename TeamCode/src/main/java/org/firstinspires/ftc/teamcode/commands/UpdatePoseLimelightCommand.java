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
 * Competitive Pose Update - MT2 ONLY Edition.
 * Corrige apenas X e Y, confiando cegamente no ângulo da Odometria/IMU.
 * Isso impede que erros de ângulo da câmera estraguem a navegação "overtime".
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

    /**
     * Hard Reset: Usa força bruta para setar ângulo e posição (se disponível).
     * Usado pelo botão de reset do piloto.
     */
    public static void forceHardReset(DrivetrainSubsystem drive, VisionSubsystem vis, double targetHeadingDegrees) {
        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);
        Pose currentPose = drive.getFollower().getPose();

        drive.getFollower().setPose(new Pose(currentPose.getX(), currentPose.getY(), targetHeadingRad));

        vis.getRobotPoseMT2(targetHeadingRad).ifPresent(mt2Pose -> {
            drive.getFollower().setPose(new Pose(
                    mt2Pose.getX(),
                    mt2Pose.getY(),
                    targetHeadingRad // Mantém o ângulo travado
            ));
            Log.i("Vision", "HARD RESET: Posição atualizada 100% via Limelight");
        });
    }

    @Override
    public void initialize() {
        if (!hasInitialized) {


            Optional<Pose> initPoseMT2 = vision.getRobotPoseMT2(fallbackPose.getHeading());

            if (initPoseMT2.isPresent()) {
                Pose p = initPoseMT2.get();
                drivetrain.getFollower().setPose(new Pose(p.getX(), p.getY(), fallbackPose.getHeading()));
                Log.d("Vision", "Inicializado via MT2 + Fallback Heading");
            } else {
                drivetrain.getFollower().setPose(fallbackPose);
                Log.w("Vision", "Câmera cega no init. Usando Fallback Pose.");
            }
            hasInitialized = true;
            return;
        }

        Pose currentPose = drivetrain.getFollower().getPose();

        vision.getRobotPoseMT2(currentPose.getHeading()).ifPresent(llPoseMT2 -> {

            double distInches = Math.hypot(llPoseMT2.getX() - currentPose.getX(), llPoseMT2.getY() - currentPose.getY());
            double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

            if (distInches < maxDeltaInches) {
                Pose fusedPose = getFusedPose(currentPose, llPoseMT2);
                drivetrain.getFollower().setPose(fusedPose);
            } else {
                Log.w("Vision", "MT2 Ignorada: Pulo muito grande (" + distInches + " in)");
            }
        });
    }

    @NonNull
    private Pose getFusedPose(Pose currentPose, Pose llPose) {
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;
        double total = wOdo + wLL;

        double fusedX = (currentPose.getX() * wOdo + llPose.getX() * wLL) / total;
        double fusedY = (currentPose.getY() * wOdo + llPose.getY() * wLL) / total;

        return new Pose(fusedX, fusedY, currentPose.getHeading());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public static void resetLocalizationStatus() {
        hasInitialized = false;
    }
}