package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command to spin the shooter motors to a specific velocity or stop them.
 * This is an instant command (finishes immediately) that just sets the state of the shooter.
 */
public class AdjustShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    private final DrivetrainSubsystem drivetrain;
    private final double targetX;
    private final double targetY;

    public AdjustShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, DrivetrainSubsystem drivetrain, double targetX, double targetY) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    @Override
    public void initialize() {
        double distance = vision.getDirectDistanceToTarget().orElse((double) 0);

        if (distance > 0) {
            distance = distance;
        } else {
            Pose pose = drivetrain.getFollower().getPose();

            double dx = targetX - pose.getX();
            double dy = targetY - pose.getY();
            double groundDistance = Math.hypot(dx, dy);

            double deltaZ = 38.75;
            double distanceInches = Math.hypot(groundDistance, deltaZ);

            distance = distanceInches / 39.3701;
        }

        double rpm = ShooterConstants.RPM_N0 + ShooterConstants.RPM_N1 * distance + ShooterConstants.RPM_N2 * Math.pow(distance, 2);

        if (distance > VisionConstants.LONGEST_DISTANCE) {
            rpm = VisionConstants.LONGEST_RPM;
        }

        shooter.setTargetVelocity(rpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}