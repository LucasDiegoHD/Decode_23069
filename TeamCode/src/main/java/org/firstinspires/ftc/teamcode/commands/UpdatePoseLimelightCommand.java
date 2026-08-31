package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Pose fallbackPose;

    public UpdatePoseLimelightCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Pose fallbackPose) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.fallbackPose = fallbackPose;
        // addRequirements(vision); <-- Removido para não travar outros comandos de visão sem querer
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
            Log.i("Vision", "HARD RESET: Posição atualizada via Limelight");
        });
    }

    @Override
    public void initialize() {
        Pose currentPose = drivetrain.getFollower().getPose();

        vision.getRobotPoseMT2(currentPose.getHeading()).ifPresent(llPoseMT2 -> {

            double distInches = Math.hypot(
                    llPoseMT2.getX() - currentPose.getX(),
                    llPoseMT2.getY() - currentPose.getY()
            );
            double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

            // CASO 1: O robô acabou de ligar (Está literalmente no 0,0)
            if (Math.abs(currentPose.getX()) < 0.1 && Math.abs(currentPose.getY()) < 0.1) {
                drivetrain.getFollower().setPose(new Pose(llPoseMT2.getX(), llPoseMT2.getY(), fallbackPose.getHeading()));
                Log.i("Vision", "Primeira inicialização via Limelight (Ignorando limite de pulo)");
            }
            // CASO 2: O robô já está andando. Só atualiza se o pulo for pequeno!
            else if (distInches < maxDeltaInches) {
                Pose fusedPose = getFusedPose(currentPose, llPoseMT2);
                drivetrain.getFollower().setPose(fusedPose);
                Log.d("Vision", "Pose atualizada via Fusão Limelight");
            }
            // CASO 3: A Limelight mentiu (Pulo gigante)
            else {
                Log.w("Vision", "MT2 Ignorada: Pulo gigante evitado (" + distInches + " in)");
            }
        });
    }

    @NonNull
    private Pose getFusedPose(Pose currentPose, Pose llPose) {
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;
        double total = wOdo + wLL;

        // FUSÃO PURA: Só altera o X e o Y. NUNCA toca no Heading (Giroscópio) do Pinpoint!
        double fusedX = (currentPose.getX() * wOdo + llPose.getX() * wLL) / total;
        double fusedY = (currentPose.getY() * wOdo + llPose.getY() * wLL) / total;

        return new Pose(fusedX, fusedY, currentPose.getHeading());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public static void resetLocalizationStatus() {}
}