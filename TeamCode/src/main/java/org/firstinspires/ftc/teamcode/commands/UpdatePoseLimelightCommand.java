package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.Optional;

/**
 * Atualiza a pose do PedroPathing usando EXCLUSIVAMENTE a pose da Limelight
 * (sem pesos), porém rejeita estimativas se forem outliers (>1m)
 * e aplica heading de MT1; se não disponível, usa heading da odometria.
 */
public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Pose initialPose;

    private static final double MAX_DELTA_INCHES = 39.73; // 1 metro

    public UpdatePoseLimelightCommand(
            DrivetrainSubsystem drivetrain,
            VisionSubsystem vision,
            Pose initialPose
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.initialPose = initialPose;
        addRequirements(vision);
    }

    public UpdatePoseLimelightCommand(
            DrivetrainSubsystem drivetrain,
            VisionSubsystem vision
    ) {
        this(drivetrain, vision, drivetrain.getFollower().getPose());
    }

    @Override
    public void initialize() {

        // Pose atual da odometria
        drivetrain.getFollower().setPose(initialPose);
        Pose currentPose = drivetrain.getFollower().getPose();

        // 1) tenta pegar posição MT2 (+ heading MT1 se possível)
        Optional<Pose> optMT2 = vision.getRobotPose();
        if (vision.getDirectDistanceToTarget().orElse((double) 0) > VisionConstants.LONGEST_DISTANCE) {
            return;
        }
        if (optMT2.isEmpty()) {
            // Sem tag -> mantém pose atual
            drivetrain.getFollower().setPose(currentPose);
            return;
        }

        Pose llPoseMT2 = optMT2.get();

        // 2) tenta pegar apenas heading de MT1
        Optional<Pose> optMT1HeadingOnly = vision.getRobotPose(llPoseMT2.getHeading());

        double finalHeading = llPoseMT2.getHeading(); // fallback padrão = heading MT2

        // Usa heading do MegaTag1 se disponível
        // fallback final: usa heading da odometria
        finalHeading = optMT1HeadingOnly.map(Pose::getHeading).orElseGet(currentPose::getHeading);

        // 3) valida distância (rejeição de outlier)
        double dx = Math.abs(llPoseMT2.getX() - currentPose.getX());
        double dy = Math.abs(llPoseMT2.getY() - currentPose.getY());

        if (dx > MAX_DELTA_INCHES || dy > MAX_DELTA_INCHES) {
            drivetrain.getFollower().setPose(currentPose);
            return;
        }

        // 4) monta pose final: posição MT2 + heading (MT1 → odo fallback)
        Pose finalPose = new Pose(
                llPoseMT2.getX(),
                llPoseMT2.getY(),
                finalHeading
        );

        drivetrain.getFollower().setPose(finalPose);
    }

    @Override
    public boolean isFinished() {
        return true; // comando executa só uma vez
    }
}
