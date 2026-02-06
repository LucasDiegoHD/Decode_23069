package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Optional;

public class HuskySubsystem extends SubsystemBase {

    private final HuskyLens huskyLens;
    private final TelemetryManager telemetry;
    private HuskyLens.Block lastValidBlock = null;

    public HuskySubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        this.huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "ERRO: Não conectado!");
        } else {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }
    }

    /**
     * Retorna o Artifact mais relevante (mais próximo) com o ID especificado.
     * Retorna um Optional vazio se nada for encontrado.
     */
    public Optional<HuskyLens.Block> getClosestAnyArtifact() {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        HuskyLens.Block bestBlock = null;
        double maxArea = 0;

        for (HuskyLens.Block block : blocks) {
            if (block.id!= HuskyConstants.COLOR_ID_PURPLE &&
                    block.id!= HuskyConstants.COLOR_ID_GREEN) {
                continue;
            }

            // 2. Filtro de Ruído (Tamanho mínimo)
            if (block.width < 15 || block.height < 15) continue;

            // 3. Filtro de Geometria (Deve parecer uma bola quadrada)
            double ratio = (double) block.width / block.height;
            if (ratio < 0.6 || ratio > 1.6) continue;

            // 4. Critério: Escolhe o objeto com maior área (mais próximo)
            // Isso faz com que o robô ignore uma bola verde longe se tiver uma roxa perto
            double area = block.width * block.height;
            if (area > maxArea) {
                maxArea = area;
                bestBlock = block;
            }
        }

        if (bestBlock!= null) {
            lastValidBlock = bestBlock;
            return Optional.of(bestBlock);
        }

        return Optional.empty();
    }

    /**
     * Calcula a distância estimada em polegadas usando a largura do objeto.
     */
    public double getDistanceToBlock(HuskyLens.Block block) {
        if (block == null) return 0;
        // Z = (W_real * f) / W_pixel
        return (HuskyConstants.ARTIFACT_REAL_WIDTH_INCHES * HuskyConstants.FOCAL_LENGTH_PIXELS) / block.width;
    }

    @Override
    public void periodic() {
        // Telemetria de debug para ajudar você a calibrar
        if (lastValidBlock!= null) {
            telemetry.addData("Husky Detectado", "ID: " + lastValidBlock.id);
            telemetry.addData("Husky Largura (px)", lastValidBlock.width);
            telemetry.addData("Husky Distancia (pol)", getDistanceToBlock(lastValidBlock));
        } else {
            telemetry.addData("Husky", "Procurando...");
        }
    }
}