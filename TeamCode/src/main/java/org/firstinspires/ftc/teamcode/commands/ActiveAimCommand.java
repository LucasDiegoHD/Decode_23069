package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
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
    private final Follower follower;
    private final double targetX;
    private final double targetY;
    private final BooleanSupplier isReadyToSpin;

    private static final double RPM_FORWARD_MULT = -6.0;
    private static final double RPM_BACKWARD_MULT = 7.0;
    private static final double MAX_SAFE_RPM = 3800;
    private static final double MIN_SAFE_RPM = 1000.0;
    private static final double TIME_OF_FLIGHT = 0.4;
    private static final double DELTA_Z = 38.75;
    private static final double INCHES_TO_METERS = 1.0 / 39.3701;

    public ActiveAimCommand(ShooterSubsystem shooter, VisionSubsystem vision, DrivetrainSubsystem drivetrain, double targetX, double targetY, BooleanSupplier isReadyToSpin) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.follower = drivetrain.getFollower();
        this.targetX = targetX;
        this.targetY = targetY;
        this.isReadyToSpin = isReadyToSpin;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        double velX = velocity.getXComponent();
        double velY = velocity.getYComponent();

        double virtualX = targetX - (velX * TIME_OF_FLIGHT);
        double virtualY = targetY - (velY * TIME_OF_FLIGHT);

        double dx = virtualX - pose.getX();
        double dy = virtualY - pose.getY();
        double groundDistance = Math.hypot(dx, dy);

        double virtualDistanceMeters = Math.hypot(groundDistance, DELTA_Z) * INCHES_TO_METERS;

        boolean robotStopped = drivetrain.isRobotStopped();

        double distanceToUse = robotStopped
                ? vision.getDirectDistanceToTarget().orElse(virtualDistanceMeters)
                : virtualDistanceMeters;

        boolean isLongShot = distanceToUse > VisionConstants.LONGEST_DISTANCE;

        double hood;
        double finalRpm;

        if (isLongShot) {
            hood = VisionConstants.LONGEST_HOOD;
            finalRpm = VisionConstants.LONGEST_RPM;
        } else {
            hood = ShooterConstants.HOOD_N0 + distanceToUse * (ShooterConstants.HOOD_N1 + distanceToUse * (ShooterConstants.HOOD_N2 + distanceToUse * ShooterConstants.HOOD_N3));
            finalRpm = ShooterConstants.RPM_N0 + distanceToUse * (ShooterConstants.RPM_N1 + distanceToUse * ShooterConstants.RPM_N2);
        }

        shooter.setLongShotMode(isLongShot);
        shooter.setHoodPosition(hood);

        if (!robotStopped && groundDistance > 1e-6 && !isLongShot) {
            double invGroundDistance = 1.0 / groundDistance;
            double unitX = dx * invGroundDistance;
            double unitY = dy * invGroundDistance;

            double speedDot = velX * unitX + velY * unitY;

            double lateralVelX = velX - (speedDot * unitX);
            double lateralVelY = velY - (speedDot * unitY);
            double lateralSpeed = Math.hypot(lateralVelX, lateralVelY);

            double effectiveDistanceMeters = Math.hypot(
                    Math.hypot(groundDistance, lateralSpeed * TIME_OF_FLIGHT),
                    DELTA_Z
            ) * INCHES_TO_METERS;

            finalRpm = ShooterConstants.RPM_N0 + effectiveDistanceMeters * (ShooterConstants.RPM_N1 + effectiveDistanceMeters * ShooterConstants.RPM_N2);

            finalRpm += speedDot > 0
                    ? speedDot * RPM_FORWARD_MULT
                    : Math.abs(speedDot) * RPM_BACKWARD_MULT;
        }
        if (distanceToUse > 3.10) {
            finalRpm += 30.0;
        }

        finalRpm += shooter.getLiveRpmOffset();

        finalRpm = Math.max(MIN_SAFE_RPM, Math.min(finalRpm, MAX_SAFE_RPM));

        shooter.setCurrentDistance(distanceToUse);

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