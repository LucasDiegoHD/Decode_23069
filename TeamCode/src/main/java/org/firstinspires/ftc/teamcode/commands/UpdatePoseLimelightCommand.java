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


        if (vision.getDirectDistanceToTarget().get() > VisionConstants.LONGEST_DISTANCE) {
            return;
        }


        // 2) tenta pegar apenas heading de MT1
        Optional<Pose> poseOptional = vision.getRobotPose(currentPose.getHeading());
        if (!poseOptional.isPresent()) {
            return;
        }
        Pose pose = poseOptional.get();

        // 3) valida distância (rejeição de outlier)
        double dx = Math.abs(pose.getX() - currentPose.getX());
        double dy = Math.abs(pose.getY() - currentPose.getY());

        if (dx > MAX_DELTA_INCHES || dy > MAX_DELTA_INCHES) {
            drivetrain.getFollower().setPose(currentPose);
            return;
        }

        // 4) monta pose final: posição MT2 + heading (MT1 → odo fallback)
        Pose finalPose = new Pose(
                pose.getX(),
                pose.getY(),
                pose.getHeading()
        );

        drivetrain.getFollower().setPose(finalPose);
    }

    @Override
    public boolean isFinished() {
        return true; // comando executa só uma vez
    }
}
