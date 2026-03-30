package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AdjustShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final double targetX;
    private final double targetY;

    private static final double RPM_FORWARD_MULT = 6.0;  // Tira pouco RPM atacando
    private static final double RPM_BACKWARD_MULT = 60.0; // Coloca mais RPM fugindo (Ajuste isso!)

    private static final double MAX_SAFE_RPM = 4500;
    private static final double MIN_SAFE_RPM = 1000.0;

    public AdjustShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, DrivetrainSubsystem drivetrain, double targetX, double targetY) {
        this.shooter = shooter;
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    @Override
    public void initialize() {
        double distance = vision.getDirectDistanceToTarget().orElse(0.0);

        Pose pose = drivetrain.getFollower().getPose();
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double groundDistance = Math.hypot(dx, dy);

        if (distance <= 0.1) {
            double deltaZ = 38.75;
            double distanceInches = Math.hypot(groundDistance, deltaZ);
            distance = distanceInches / 39.3701;
        }

        double baseRpm = ShooterConstants.RPM_N0 + ShooterConstants.RPM_N1 * distance + ShooterConstants.RPM_N2 * Math.pow(distance, 2);

        Vector velocity = drivetrain.getFollower().getVelocity();
        double velTowardsGoal = 0.0;

        if (groundDistance > 1.0) {
            double dirX = dx / groundDistance;
            double dirY = dy / groundDistance;
            velTowardsGoal = (velocity.getXComponent() * dirX) + (velocity.getYComponent() * dirY);
        }

        double rpmAdjustment = 0;
        if (velTowardsGoal > 0) {
            rpmAdjustment = velTowardsGoal * RPM_FORWARD_MULT;
        } else {
            rpmAdjustment = velTowardsGoal * RPM_BACKWARD_MULT;
        }

        double finalRpm = baseRpm - rpmAdjustment;

        if (distance > VisionConstants.LONGEST_DISTANCE) {
            finalRpm = VisionConstants.LONGEST_RPM;
        }

        finalRpm = Math.max(MIN_SAFE_RPM, Math.min(finalRpm, MAX_SAFE_RPM));
        shooter.setTargetVelocity(finalRpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}