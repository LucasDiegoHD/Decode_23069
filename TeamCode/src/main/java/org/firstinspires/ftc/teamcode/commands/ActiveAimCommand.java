package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;

public class ActiveAimCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final double targetX;
    private final double targetY;
    private final BooleanSupplier isReadyToSpin;
    private static final double RPM_FORWARD_MULT = -6.0;
    private static final double RPM_BACKWARD_MULT = 7.0;
    private static final double MAX_SAFE_RPM = 3800;
    private static final double MIN_SAFE_RPM = 1000.0;
    private static final double TIME_OF_FLIGHT = 0.4;

    public ActiveAimCommand(ShooterSubsystem shooter, VisionSubsystem vision, DrivetrainSubsystem drivetrain, double targetX, double targetY, BooleanSupplier isReadyToSpin) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;
        this.isReadyToSpin = isReadyToSpin;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Pose pose = drivetrain.getFollower().getPose();
        Vector velocity = drivetrain.getFollower().getVelocity();

        double virtualX = targetX - (velocity.getXComponent() * TIME_OF_FLIGHT);
        double virtualY = targetY - (velocity.getYComponent() * TIME_OF_FLIGHT);

        double dx = virtualX - pose.getX();
        double dy = virtualY - pose.getY();
        double groundDistance = Math.hypot(dx, dy);

        double deltaZ = 38.75;
        double virtualDistanceInches = Math.hypot(groundDistance, deltaZ);
        double virtualDistanceMeters = virtualDistanceInches / 39.3701;

        double distanceToUse;
        if (drivetrain.isRobotStopped()) {
            distanceToUse = vision.getDirectDistanceToTarget().orElse(virtualDistanceMeters);
        } else {
            distanceToUse = virtualDistanceMeters;
        }

        double hood = ShooterConstants.HOOD_N0 + ShooterConstants.HOOD_N1 * distanceToUse
                + ShooterConstants.HOOD_N2 * Math.pow(distanceToUse, 2) + ShooterConstants.HOOD_N3 * Math.pow(distanceToUse, 3);

        double finalRpm = ShooterConstants.RPM_N0 + ShooterConstants.RPM_N1 * distanceToUse
                + ShooterConstants.RPM_N2 * Math.pow(distanceToUse, 2);

        if (distanceToUse > VisionConstants.LONGEST_DISTANCE) {
            hood = VisionConstants.LONGEST_HOOD;
            finalRpm = VisionConstants.LONGEST_RPM;
        }

        shooter.setLongShotMode(distanceToUse > VisionConstants.LONGEST_DISTANCE);
        shooter.setHoodPosition(hood);

        if (!drivetrain.isRobotStopped() && groundDistance > 1e-6 && distanceToUse < VisionConstants.LONGEST_DISTANCE) {
            double unitX = dx / groundDistance;
            double unitY = dy / groundDistance;

            double speedDot = velocity.getXComponent() * unitX + velocity.getYComponent() * unitY;

            if (speedDot > 0) {
                finalRpm += speedDot * RPM_FORWARD_MULT;
            } else {
                finalRpm += Math.abs(speedDot) * RPM_BACKWARD_MULT;
            }
        }

        finalRpm = Math.max(MIN_SAFE_RPM, Math.min(finalRpm, MAX_SAFE_RPM));

        if (isReadyToSpin.getAsBoolean()) {
            shooter.setTargetVelocity(finalRpm);
        } else {
            shooter.setTargetVelocity(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}