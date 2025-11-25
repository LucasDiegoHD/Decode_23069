package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.Optional;

public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private Pose initialPose;

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
        this(drivetrain, vision, null);
    }

    @Override
    public void initialize() {

        if (initialPose == null) {
            initialPose = drivetrain.getFollower().getPose();
        } else {
            drivetrain.getFollower().setPose(initialPose);
        }

        Log.d("UpdatePose", "initialize: " + initialPose);

        Pose currentPose = drivetrain.getFollower().getPose();

        Optional<Pose> poseOptional = vision.getRobotPose(currentPose.getHeading());

        if (poseOptional.isEmpty()) {
            drivetrain.getFollower().setPose(currentPose);
            return;
        }

        Pose llPose = poseOptional.get();

        // ---- Filtro: rejeita se estiver muito longe ----
        double dx = llPose.getX() - currentPose.getX();
        double dy = llPose.getY() - currentPose.getY();
        double distInches = Math.hypot(dx, dy);

        double maxDeltaInches =
                VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

        if (distInches > maxDeltaInches) {
            Log.d("UpdatePose", "LL rejeitada: dist=" + distInches);
            drivetrain.getFollower().setPose(currentPose);
            return;
        }

        // ---- Fusão com pesos ----
        Pose fusedPose = getPose(currentPose, llPose);

        drivetrain.getFollower().setPose(fusedPose);
        Log.d("UpdatePose", "Pose LL=" + llPose + " | Fused=" + fusedPose);
    }

    @NonNull
    private static Pose getPose(Pose currentPose, Pose llPose) {
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;

        double sum = wOdo + wLL;
        if (sum == 0) {
            wOdo = 1;
            wLL = 0;
        } else {
            wOdo /= sum;
            wLL /= sum;
        }

        double fusedX = currentPose.getX() * wOdo + llPose.getX() * wLL;
        double fusedY = currentPose.getY() * wOdo + llPose.getY() * wLL;
        double fusedHeading = currentPose.getHeading() * wOdo + llPose.getHeading() * wLL;

        Pose fusedPose = new Pose(fusedX, fusedY, fusedHeading);
        return fusedPose;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
