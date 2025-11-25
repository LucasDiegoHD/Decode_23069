package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * VisionFusionCommand (Comando de Fusão de Sensores)
 * * Versão Melhorada: Inclui correção suave de Heading (Ângulo) e Telemetria de Estado.
 */
public class VisionFusionCommand extends CommandBase {

    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private final Follower follower;

    public VisionFusionCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TelemetryManager telemetry) {
        this.vision = vision;
        this.telemetry = telemetry;
        this.follower = drivetrain.getFollower();
        addRequirements(vision);
    }

    @Override
    public void execute() {

        Pose poseOdometria = follower.getPose();

        if (vision.hasTarget()) {

            // Passamos o heading da odometria para ajudar o MegaTag2, mas agora também vamos ler o retorno dele
            vision.getRobotPose(poseOdometria.getHeading()).ifPresent(poseCamera -> {

                // 3. Filtro de Segurança
                double diferencaEntreSensores = Math.hypot(
                        poseCamera.getX() - poseOdometria.getX(),
                        poseCamera.getY() - poseOdometria.getY()
                );

                if (diferencaEntreSensores < VisionConstants.MAX_FUSION_ERROR) {

                    // 4. Calcular Pesos baseados na velocidade
                    Vector vetorVelocidade = follower.getVelocity();
                    double velocidadeAtual = vetorVelocidade.getMagnitude();

                    double pesoDaVisao;
                    String modoFusao; // Para debug

                    if (velocidadeAtual > VisionConstants.FUSION_SPEED_THRESHOLD) {
                        // MODO RÁPIDO
                        pesoDaVisao = VisionConstants.FUSION_WEIGHT_FAST;
                        modoFusao = "FAST (Low Weight)";
                    } else {
                        // MODO LENTO / MIRANDO
                        pesoDaVisao = VisionConstants.FUSION_WEIGHT_SLOW;
                        modoFusao = "SLOW (High Weight)";
                    }

                    // 5. Média Ponderada para X e Y
                    double novoX = poseOdometria.getX() + (poseCamera.getX() - poseOdometria.getX()) * pesoDaVisao;
                    double novoY = poseOdometria.getY() + (poseCamera.getY() - poseOdometria.getY()) * pesoDaVisao;

                    //  Fusão de Heading (Ângulo)
                    // O IMU drifta com o tempo. Usamos um peso BEM PEQUENO (ex: 0.01 fixo ou metade do peso da visão)
                    // para corrigir o ângulo devagarinho, mantendo o robô alinhado com o campo.
                    double headingOdo = poseOdometria.getHeading();
                    double headingCam = poseCamera.getHeading();

                    // Calcula a menor diferença angular (para não girar 360 do nada)
                    double erroAngular = angleDifference(headingCam, headingOdo);

                    // Usamos um peso conservador (ex: 0.02) para o ângulo, pois a câmera pode tremer a rotação
                    double pesoAngular = 0.02;
                    double novoHeading = headingOdo + (erroAngular * pesoAngular);

                    // Criamos a pose final "Fundida" com Heading corrigido
                    Pose poseFundida = new Pose(novoX, novoY, novoHeading);

                    // 6. Atualizar o Pedro Pathing
                    follower.setPose(poseFundida);

                    telemetry.debug("Fusion/Mode", modoFusao);
                    telemetry.debug("Fusion/Alpha", pesoDaVisao);
                    telemetry.debug("Fusion/Delta Pos", diferencaEntreSensores);
                    telemetry.debug("Fusion/Delta Ang (Deg)", Math.toDegrees(erroAngular));
                } else {
                    telemetry.debug("Fusion/Status", "IGNORADO - Erro > MAX (Possível Glare)");
                }
            });
        } else {
            telemetry.debug("Fusion/Status", "No Target");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Calcula a menor diferença entre dois ângulos (em radianos).
     * Garante que o robô não tente corrigir girando o caminho mais longo.
     */
    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}