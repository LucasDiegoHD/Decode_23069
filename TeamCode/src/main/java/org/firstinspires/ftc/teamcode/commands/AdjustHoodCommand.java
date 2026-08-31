package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command to adjust the hood position based on distance.
 * This is an instant command (finishes immediately) that just sets the state of the hood.
 */
public class AdjustHoodCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    private final DrivetrainSubsystem drivetrain;
    private final double targetX;
    private final double targetY;

    /**
     * Creates a new AdjustHoodCommand.
     *
     * @param shooter The ShooterSubsystem to control.
     */
    public AdjustHoodCommand(ShooterSubsystem shooter, VisionSubsystem vision, DrivetrainSubsystem drivetrain, double targetX, double targetY) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    /**
     * Called when the command is initially scheduled. Executes the specified hood action.
     */
    @Override
    public void initialize() {
        double distance = vision.getDirectDistanceToTarget().orElse((double) 0);

        if (distance > 0) {
        } else {

            Pose pose = drivetrain.getFollower().getPose();
            double dx = targetX - pose.getX();
            double dy = targetY - pose.getY();

            double groundDistance = Math.hypot(dx, dy);

            double deltaZ = 28.5;

            double distanceInches = Math.hypot(groundDistance, deltaZ);

            distance = distanceInches / 39.3701;
        }

        double hood = ShooterConstants.HOOD_N0 + ShooterConstants.HOOD_N1 * distance
                + ShooterConstants.HOOD_N2 * Math.pow(distance, 2) + ShooterConstants.HOOD_N3 * Math.pow(distance, 3);

        if (distance > VisionConstants.LONGEST_DISTANCE) {
            hood = VisionConstants.LONGEST_HOOD;
        }

        boolean longShotMode = distance > VisionConstants.LONGEST_DISTANCE;
        shooter.setLongShotMode(longShotMode);

        shooter.setHoodPosition(hood);
    }

    /**
     * Returns true when the command should end.
     *
     * @return True immediately, as this is an instant command.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}