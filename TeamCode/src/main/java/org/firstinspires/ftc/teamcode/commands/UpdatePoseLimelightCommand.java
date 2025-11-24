package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * Atualiza a pose do PedroPathing usando a pose da Limelight,
 * porém rejeita estimativas da LL se estiverem muito longe (>2m)
 * ou se a pose da Limelight for nula.
 */
public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Pose initialPose;

    private static final double MAX_DELTA_INCHES = 39.73; // 1  metros

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

        drivetrain.getFollower().setPose(initialPose);

        Pose currentPose = drivetrain.getFollower().getPose();

        vision.getRobotPose().ifPresentOrElse(llPoseRaw -> {

            vision.getRobotPose(llPoseRaw.getHeading()).ifPresentOrElse(llPose -> {

                double dx = Math.abs(llPose.getX() - currentPose.getX());
                double dy = Math.abs(llPose.getY() - currentPose.getY());

                if (dx > MAX_DELTA_INCHES || dy > MAX_DELTA_INCHES) {
                    drivetrain.getFollower().setPose(currentPose);
                    return;
                }

                drivetrain.getFollower().setPose(llPose);

            }, () -> {
                drivetrain.getFollower().setPose(currentPose);
            });

        }, () -> {
            drivetrain.getFollower().setPose(currentPose);
        });
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
    