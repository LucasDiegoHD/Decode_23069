package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import com.pedropathing.geometry.Pose;

import java.util.Optional;

public class VisionFusionCommand extends CommandBase {

    private final DrivetrainSubsystem drive;
    private final VisionSubsystem vision;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean pendingUpdate = false;

    // Última pose filtrada da limelight (EMA)
    private Pose lastFilteredLLPose = null;

    public VisionFusionCommand(DrivetrainSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(vision); // não trava nenhum subsystem, fica paralelo
    }

    @Override
    public void initialize() {
        timer.reset();
        pendingUpdate = false;
    }

    @Override
    public void execute() {

        double elapsed = timer.seconds();

        // 🟢 Chegou a hora de tentar atualizar
        if (elapsed >= VisionConstants.UPDATE_INTERVAL_SECONDS) {
            pendingUpdate = true;
        }

        // Se não está no modo de "tentar atualizar", não faz nada
        if (!pendingUpdate)
            return;

        // 🟡 Tenta pegar a pose da limelight (com seu subsistema atual)
        Optional<Pose> llOpt = vision.getRobotPose();
        if (llOpt.isEmpty()) {
            // ❌ Se não conseguiu, tenta de novo no próximo loop (SEM esperar 2s)
            return;
        }
        if (vision.getDirectDistanceToTarget().orElse((double) 0) > VisionConstants.LONGEST_DISTANCE) {
            return;
        }

        Pose llPose = llOpt.get();
        Pose odoPose = drive.getFollower().getPose();

        // 🟡 Filtro EMA para suavizar limelight
        llPose = applyEMAFilter(llPose);

        // 🛑 Rejeição de outliers (se a leitura é muito longe da odometria)
        if (distanceBetween(llPose, odoPose) > VisionConstants.MAX_ALLOWED_JUMP) {
            return;
        }

        // 🟢 Fusão final
        Pose fused = fusePoses(odoPose, llPose);

        // Aplicar no drive
        drive.getFollower().setPose(fused);

        // Resetar atualização
        pendingUpdate = false;
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return false; // comando roda para sempre
    }

    // -------------------------------------------------------------
    //  FILTRO EMA — suaviza limelight
    // -------------------------------------------------------------
    private Pose applyEMAFilter(Pose newPose) {

        double alpha = VisionConstants.EMA_ALPHA;

        if (lastFilteredLLPose == null) {
            lastFilteredLLPose = newPose;
            return newPose;
        }

        double x = alpha * newPose.getX() + (1 - alpha) * lastFilteredLLPose.getX();
        double y = alpha * newPose.getY() + (1 - alpha) * lastFilteredLLPose.getY();
        double h = alpha * newPose.getHeading() + (1 - alpha) * lastFilteredLLPose.getHeading();

        lastFilteredLLPose = new Pose(x, y, h);
        return lastFilteredLLPose;
    }

    // -------------------------------------------------------------
    //  FUSÃO DE POSES — com pesos adaptativos
    // -------------------------------------------------------------
    private Pose fusePoses(Pose odo, Pose ll) {

        double wLL = VisionConstants.BASE_WEIGHT_LIMELIGHT;
        double wOdo = VisionConstants.BASE_WEIGHT_ODOMETRY;

        // 🔵 Peso adaptativo (aumenta peso da limelight quando erro é grande)
        if (VisionConstants.ENABLE_ADAPTIVE_WEIGHT) {
            double dist = distanceBetween(odo, ll);
            double scale = Math.min(1.0, dist / VisionConstants.ADAPTIVE_ERROR_SCALE);

            wLL += 0.3 * scale;
            wOdo = 1.0 - wLL;
        }

        double x = odo.getX() * wOdo + ll.getX() * wLL;
        double y = odo.getY() * wOdo + ll.getY() * wLL;
        double h = odo.getHeading() * wOdo + ll.getHeading() * wLL;

        return new Pose(x, y, h);
    }

    // -------------------------------------------------------------
    //  Cálculo de distância
    // -------------------------------------------------------------
    private double distanceBetween(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
