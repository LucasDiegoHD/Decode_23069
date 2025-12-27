package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.Optional;


/**
 * The VisionSubsystem is responsible for handling all vision-related tasks,
 * including target detection and pose estimation using a Limelight camera.
 */
//@AutoLog
public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    private static final double INCHES_IN_METER = 39.3701;

    /**
     * Constructs a new VisionSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     */
    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    /**
     * Gets the horizontal offset from the crosshair to the target.
     * @return An Optional containing the 'tx' value if a target is present.
     */
    public Optional<Double> getTargetTx() {
        if (hasTarget()) {
            return Optional.of(latestResult.getTx());
        }
        return Optional.empty();
    }

    /**
     * Checks if the Limelight has a valid target.
     * @return True if a valid target is detected, false otherwise.
     */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }


    public Optional<Pose> getRobotPose() {
        latestResult = limelight.getLatestResult();

        if (!hasTarget()) {
            return Optional.empty();
        }
        Pose3D robotPose = latestResult.getBotpose();
        if (robotPose == null) {
            return Optional.empty();
        }
        // Convert from meters (Limelight standard) to inches (PedroPathing standard)
        return Optional.of(FTCCoordinates.INSTANCE.convertToPedro(new Pose(robotPose.getPosition().x * INCHES_IN_METER, robotPose.getPosition().y * INCHES_IN_METER, Math.toRadians(robotPose.getOrientation().getYaw()))));
    }

    /**
     * Obtém a Pose usando MegaTag 1 (BotPose padrão).
     * Útil para inicialização onde o Heading do robô pode estar incerto.
     */
    public Optional<Pose> getMegaTag1Pose() {
        // latestResult é atualizado no periodic(), mas para garantir dados frescos no comando:
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return Optional.empty();

        Pose3D botPose = result.getBotpose(); // MT1 Standard
        if (botPose == null) return Optional.empty();

        return Optional.of(convertLLToPedro(botPose));
    }

    /**
     * Obtém a Pose usando MegaTag 2.
     * Requer que o Heading do robô esteja sendo enviado corretamente (updateLimelightYaw).
     * Mais robusto contra ambiguidade de tags.
     */
    public Optional<Pose> getMegaTag2Pose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return Optional.empty();

        Pose3D botPoseMT2 = result.getBotpose_MT2(); // MT2
        if (botPoseMT2 == null) return Optional.empty();

        return Optional.of(convertLLToPedro(botPoseMT2));
    }

    /**
     * Helper para converter coordenadas da Limelight (Metros, Sistema LL)
     * para Pedro Pathing (Polegadas, Sistema FTC).
     */
    private Pose convertLLToPedro(Pose3D llPose) {
        // A Limelight retorna em Metros. Pedro usa Polegadas.
        // O FTCCoordinates lida com a rotação do sistema de coordenadas.
        return FTCCoordinates.INSTANCE.convertToPedro(
                new Pose(
                        llPose.getPosition().x * INCHES_IN_METER,
                        llPose.getPosition().y * INCHES_IN_METER,
                        Math.toRadians(llPose.getOrientation().getYaw())
                )
        );
    }

    /**
     * Calculates the direct distance (hypotenuse) from the camera to the target.
     * @return An {@code Optional<Double>} containing the distance in meters.
     */
    public Optional<Double> getDirectDistanceToTarget() {
        if (!hasTarget()) {
            return Optional.empty();
        }

        if(latestResult.getFiducialResults().get(0) != null) {
            // The z-position in camera space is the direct distance to the target
            return Optional.of(Math.abs(latestResult.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().z));
        }

        return Optional.empty();

    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();

        updateLimelightYaw(drivetrain.getFollower().getHeading());

        if (latestResult != null) {
            getDirectDistanceToTarget().ifPresent(distance -> telemetry.addData("Distância Direta (M)", distance));
        } else {
            telemetry.addLine("LL sem resultado");
        }
    }

    public void updateLimelightYaw(double yaw){
        limelight.updateRobotOrientation(Math.toDegrees(yaw));
    }
}
