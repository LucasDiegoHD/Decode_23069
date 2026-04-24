package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private LLResult latestResult;
    private final TelemetryManager telemetry;
    private static final double INCHES_IN_METER = 39.3701;

    public VisionSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void periodic() {
        // Ponto único de leitura da Limelight por loop.
        // Todos os métodos usam this.latestResult — zero chamadas extras ao hardware.
        latestResult = limelight.getLatestResult();

        if (latestResult != null && latestResult.isValid()) {
            telemetry.addData("Limelight", "👀 Vendo Alvo");
        } else {
            telemetry.addData("Limelight", "❌ Cega");
        }
    }

    public Optional<Pose> getRobotPoseMT1() {
        if (!hasTarget()) return Optional.empty();
        Pose3D botPose = latestResult.getBotpose();
        if (botPose == null) return Optional.empty();
        return Optional.of(convertToPedro(botPose));
    }

    public Optional<Pose> getRobotPoseMT2(double yawRadians) {
        // updateRobotOrientation envia o yaw para o próximo frame da Limelight.
        // Como ela é assíncrona, o comportamento é equivalente ao original.
        limelight.updateRobotOrientation(Math.toDegrees(yawRadians) + 90);
        if (!hasTarget()) return Optional.empty();
        Pose3D botPose = latestResult.getBotpose_MT2();
        if (botPose == null) return Optional.empty();
        return Optional.of(convertToPedro(botPose));
    }

    public Optional<Double> getTargetTx() {
        if (hasTarget()) return Optional.of(latestResult.getTx());
        return Optional.empty();
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public Optional<Double> getDirectDistanceToTarget() {
        if (!hasTarget()) return Optional.empty();
        if (!latestResult.getFiducialResults().isEmpty()) {
            return Optional.of(
                    Math.abs(
                            latestResult.getFiducialResults().get(0)
                                    .getTargetPoseCameraSpace().getPosition().z
                    )
            );
        }
        return Optional.empty();
    }

    public void updateLimelightYaw(double yaw) {
        limelight.updateRobotOrientation(Math.toDegrees(yaw));
    }

    public Optional<Pose> getRobotPose(double yaw) {
        return getRobotPoseMT2(yaw);
    }

    public Optional<Pose> getRobotPose() {
        if (!hasTarget()) return Optional.empty();
        Pose3D robotPose = latestResult.getBotpose();
        if (robotPose == null) return Optional.empty();
        return Optional.of(convertToPedro(robotPose));
    }

    private Pose convertToPedro(Pose3D pose3d) {
        Pose rawPose = new Pose(
                pose3d.getPosition().x * INCHES_IN_METER,
                pose3d.getPosition().y * INCHES_IN_METER,
                Math.toRadians(pose3d.getOrientation().getYaw())
        );
        return FTCCoordinates.INSTANCE.convertToPedro(rawPose);
    }
}