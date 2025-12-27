package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime; // Importante para o timer

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final BooleanSupplier updatePosition;

    // Timer para controlar a transição MT1 -> MT2
    private final ElapsedTime timer = new ElapsedTime();

    // Flag para saber se já fizemos o "Snap" inicial (teleporte para a posição da câmera)
    // Static para persistir entre agendamentos se necessário, mas cuidado se reiniciar o OpMode
    private static boolean hasInitialized = false;

    public UpdatePoseLimelightCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, BooleanSupplier updatePosition) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.updatePosition = updatePosition;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        // Reinicia o timer assim que o comando começa
        timer.reset();
    }

    @Override
    public void execute() {
        // 1. Verifica se o boolean permite a atualização (ex: não atualizar se estiver rodando rápido demais ou girando)
        if (!updatePosition.getAsBoolean()) {
            return;
        }

        Optional<Pose> visionPoseOptional;

        // 2. Lógica de Tempo: Primeiros 3s usa MT1, depois usa MT2
        if (timer.seconds() < 3.0) {
            visionPoseOptional = vision.getMegaTag1Pose();
        } else {
            visionPoseOptional = vision.getMegaTag2Pose();
        }

        // Se não tiver tag, encerra este ciclo
        if (visionPoseOptional.isEmpty()) {
            return;
        }

        Pose llPose = visionPoseOptional.get();
        Pose currentPose = drivetrain.getFollower().getPose();

        // 3. Initial Snap: Se nunca inicializamos, confiamos 100% na câmera na primeira vez
        // Isso evita que o robô comece em (0,0) se você não usar a pose de fallback
        if (!hasInitialized) {
            drivetrain.getFollower().setPose(llPose);
            hasInitialized = true;
            Log.d("Vision", "Initial Snap Successful (MT1): " + llPose);
            return;
        }

        // 4. Proteção contra Saltos (Reflexos/Erros)
        double distInches = Math.hypot(llPose.getX() - currentPose.getX(), llPose.getY() - currentPose.getY());
        double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * 39.37; // Converte metros para polegadas

        if (distInches > maxDeltaInches) {
            Log.w("Vision", "Large jump rejected: " + distInches + " inches.");
            return; // Ignora este frame
        }

        // 5. Fusão Ponderada (Odometria > Câmera)
        Pose fusedPose = getFusedPose(currentPose, llPose);
        drivetrain.getFollower().setPose(fusedPose);
    }

    @NonNull
    private Pose getFusedPose(Pose currentPose, Pose llPose) {
        // Seus pesos definidos em VisionConstants
        // Recomendado: ODO_WEIGHT = 0.95, LL_WEIGHT = 0.05 (confia mais no Pinpoint)
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;
        double sum = wOdo + wLL;

        if (sum == 0) return currentPose; // Evita divisão por zero

        wOdo /= sum; // Normaliza
        wLL /= sum;

        return new Pose(
                currentPose.getX() * wOdo + llPose.getX() * wLL,
                currentPose.getY() * wOdo + llPose.getY() * wLL,
                currentPose.getHeading() // MANTÉM O HEADING DO PINPOINT (Mais preciso e rápido)
        );
    }

    @Override
    public boolean isFinished() {
        // Retorna false para rodar continuamente enquanto o OpMode estiver ativo
        return false;
    }

    public static void resetLocalizationStatus() {
        hasInitialized = false;
    }
}