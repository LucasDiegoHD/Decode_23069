package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

public class KinematicAimDriveCommand extends CommandBase {

    private final Follower follower;
    private final DrivetrainSubsystem drivetrain;
    private final PIDFController turnController;
    private final GamepadEx driver;
    private final double targetX;
    private final double targetY;
    private boolean isAtTarget = false;

    private static final double ARTIFACT_VELOCITY_INCHES_PER_SEC = 900.0;
    private static final double SYSTEM_LATENCY_SECONDS = 0.4;

    private double smoothedVelX = 0.0;
    private double smoothedVelY = 0.0;
    private static final double VEL_ALPHA = 0.8;
    private static final double FEEDFORWARD_DEAD_ZONE = Math.toRadians(2.0);

    public KinematicAimDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driver, double targetX, double targetY) {
        this.follower = drivetrain.getFollower();
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.targetX = targetX;
        this.targetY = targetY;

        turnController = new PIDFController(
                ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD,
                ShooterConstants.ANGLE_KF
        );

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
        turnController.reset();
        turnController.setSetPoint(0);
        turnController.setTolerance((Math.PI / 180.0) * ShooterConstants.ANGLE_TOLERANCE);

        smoothedVelX = follower.getVelocity().getXComponent();
        smoothedVelY = follower.getVelocity().getYComponent();
        isAtTarget = false;
    }

    @Override
    public void execute() {
        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double heading = pose.getHeading();

        double rawY = driver.getLeftX();
        double rawX = driver.getLeftY();

        double targetY_input = rawY * Math.abs(rawY);
        double targetX_input = rawX * Math.abs(rawX);

        double xField = targetX_input * Math.cos(heading) - targetY_input * Math.sin(heading);
        double yField = targetX_input * Math.sin(heading) + targetY_input * Math.cos(heading);

        if (DataStorage.alliance == AllianceEnum.Blue) {
            xField = -xField;
            yField = -yField;
        }

        smoothedVelX = (VEL_ALPHA * velocity.getXComponent()) + ((1 - VEL_ALPHA) * smoothedVelX);
        smoothedVelY = (VEL_ALPHA * velocity.getYComponent()) + ((1 - VEL_ALPHA) * smoothedVelY);

        double velMagnitude = Math.hypot(smoothedVelX, smoothedVelY);
        if (velMagnitude < 2.0) {
            smoothedVelX = 0.0;
            smoothedVelY = 0.0;
        }

        double robotX = pose.getX();
        double robotY = pose.getY();

        double diffX = targetX - robotX;
        double diffY = targetY - robotY;
        double distanceToTarget = Math.hypot(diffX, diffY);
        if (distanceToTarget < 1.0) distanceToTarget = 1.0;

        double targetDirX = diffX / distanceToTarget;
        double targetDirY = diffY / distanceToTarget;
        double velTowardsGoal = (smoothedVelX * targetDirX) + (smoothedVelY * targetDirY);

        double effectiveArtifactVelocity = ARTIFACT_VELOCITY_INCHES_PER_SEC + velTowardsGoal;
        if (effectiveArtifactVelocity < 100.0) effectiveArtifactVelocity = 100.0;

        double timeOfFlight = distanceToTarget / effectiveArtifactVelocity;
        double totalPredictionTime = timeOfFlight + SYSTEM_LATENCY_SECONDS;

        double virtualX = targetX - (smoothedVelX * totalPredictionTime);
        double virtualY = targetY - (smoothedVelY * totalPredictionTime);

        double desiredAngle = Math.atan2(virtualY - robotY, virtualX - robotX);
        double error = angleDifference(desiredAngle, heading);

        double turnPower = turnController.calculate(error);

        double innerTolerance = Math.toRadians(ShooterConstants.ANGLE_TOLERANCE);
        double outerTolerance = innerTolerance + Math.toRadians(0.4);

        if (isAtTarget) {
            if (Math.abs(error) > outerTolerance) isAtTarget = false;
        } else {
            if (Math.abs(error) < innerTolerance) isAtTarget = true;
        }

        if (!isAtTarget && Math.abs(error) > FEEDFORWARD_DEAD_ZONE) {
            turnPower += Math.copySign(ShooterConstants.ANGLE_KF, turnPower);
        }

        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        follower.setTeleOpDrive(
                xField,
                -yField,
                -turnPower,
                true
        );
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}