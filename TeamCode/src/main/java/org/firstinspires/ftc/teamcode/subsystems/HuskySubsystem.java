package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
     * Retorna o Artifact mais relevante (mais próximo/maior área).
     * Retorna um Optional vazio se nada for encontrado.
     */
    public Optional<HuskyLens.Block> getClosestAnyArtifact() {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        // SEGURANÇA: Se a câmera falhar ou não ver nada, retorna vazio imediatamente
        if (blocks == null || blocks.length == 0) {
            return Optional.empty();
        }

        HuskyLens.Block bestBlock = null;
        double maxArea = -1;

        for (HuskyLens.Block block : blocks) {
            // 1. Filtro de ID (Aceita apenas as cores desejadas)
            boolean isValidId = (block.id == HuskyConstants.COLOR_ID_PURPLE ||
                    block.id == HuskyConstants.COLOR_ID_GREEN ||
                    block.id == HuskyConstants.COLOR_ID_GREEN2 ||
                    block.id == HuskyConstants.COLOR_ID_PURPLE2);

            if (!isValidId) continue;

            // 2. Filtro de Ruído (Tamanho mínimo em pixels)
            if (block.width < 15 || block.height < 15) continue;

            // 3. Filtro de Geometria (Deve parecer uma bola/quadrado e não um risco)
            double ratio = (double) block.width / block.height;
            if (ratio < 0.6 || ratio > 1.6) continue;

            // 4. Critério: Escolhe o objeto com maior área (O mais próximo da câmera)
            double area = block.width * block.height;
            if (area > maxArea) {
                maxArea = area;
                bestBlock = block;
            }
        }

        if (bestBlock != null) {
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
        // Fórmula: Z = (W_real * f) / W_pixel
        return (HuskyConstants.ARTIFACT_REAL_WIDTH_INCHES * HuskyConstants.FOCAL_LENGTH_PIXELS) / block.width;
    }

    @Override
    public void periodic() {
        // Telemetria de debug
        if (lastValidBlock != null) {
            telemetry.addData("Husky ID", lastValidBlock.id);
            telemetry.addData("Husky X", lastValidBlock.x); // Ajuda a ver o centro
            telemetry.addData("Husky Dist (in)", getDistanceToBlock(lastValidBlock));
        } else {
            telemetry.addData("Husky", "Procurando...");
        }
    }
}