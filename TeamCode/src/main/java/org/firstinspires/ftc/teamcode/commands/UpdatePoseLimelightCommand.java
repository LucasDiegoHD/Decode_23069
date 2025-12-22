package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import java.util.Optional;

/**
 * Competitive Pose Update Command.
 * Features: Snap-on-start, Tag-loss protection, and MegaTag 1 stability.
 */
public class UpdatePoseLimelightCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;
    private final Pose fallbackPose;

    private static boolean hasInitialized = false;

    public UpdatePoseLimelightCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, Pose fallbackPose) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.fallbackPose = fallbackPose;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        Optional<Pose> poseOptional = vision.getRobotPose();

        if (poseOptional.isEmpty()) {
            if (!hasInitialized) {
                // If we never synced, move to fallback so Field-Oriented isn't (0,0)
                drivetrain.getFollower().setPose(fallbackPose);
                Log.w("Vision", "No tag & not initialized. Using fallback.");
            } else {
                // If we were already synced, just trust odometry (do nothing)
                Log.d("Vision", "Tag lost. Maintaining odometry tracking.");
            }
            return;
        }

        Pose llPose = poseOptional.get();
        Pose currentPose = drivetrain.getFollower().getPose();

        if (!hasInitialized) {
            drivetrain.getFollower().setPose(llPose);
            hasInitialized = true;
            Log.d("Vision", "Initial Snap Successful: " + llPose);
            return;
        }

        double distInches = Math.hypot(llPose.getX() - currentPose.getX(), llPose.getY() - currentPose.getY());
        double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

        if (distInches > maxDeltaInches) {
            // If the jump is too big, it's likely a reflection. We DON'T reset hasInitialized.
            // We just ignore this frame to prevent the robot from "glitching".
            Log.w("Vision", "Large jump rejected: " + distInches + " inches.");
            return;
        }

        Pose fusedPose = getFusedPose(currentPose, llPose);
        drivetrain.getFollower().setPose(fusedPose);
    }

    @NonNull
    private Pose getFusedPose(Pose currentPose, Pose llPose) {
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;
        double sum = wOdo + wLL;
        if (sum == 0) return currentPose;

        wOdo /= sum;
        wLL /= sum;

        return new Pose(
                currentPose.getX() * wOdo + llPose.getX() * wLL,
                currentPose.getY() * wOdo + llPose.getY() * wLL,
                currentPose.getHeading() // Keep odometry heading
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    // Helper to reset status if needed (e.g. at start of OpMode)
    public static void resetLocalizationStatus() {
        hasInitialized = false;
    }
}