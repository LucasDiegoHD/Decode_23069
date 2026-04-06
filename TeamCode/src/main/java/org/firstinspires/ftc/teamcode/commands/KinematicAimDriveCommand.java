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

    private static final double ARTIFACT_VELOCITY_INCHES_PER_SEC = 400.0;
    private static final double SYSTEM_LATENCY_SECONDS = 0.30;

    // --- VARIÁVEIS DO FILTRO DE FLUIDEZ ---
    private double smoothedVelX = 0.0;
    private double smoothedVelY = 0.0;
    private static final double VEL_ALPHA = 0.8;

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
        turnController.reset();
        turnController.setSetPoint(0);
        turnController.setTolerance((Math.PI / 180.0) * ShooterConstants.ANGLE_TOLERANCE);

        smoothedVelX = follower.getVelocity().getXComponent();
        smoothedVelY = follower.getVelocity().getYComponent();
    }

    @Override
    public void execute() {
        double forward = -driver.getLeftX();
        double strafe = -driver.getLeftY();

        if (DataStorage.alliance == AllianceEnum.Blue) {
            forward = -forward;
            strafe = -strafe;
        }

        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        smoothedVelX = (VEL_ALPHA * velocity.getXComponent()) + ((1 - VEL_ALPHA) * smoothedVelX);
        smoothedVelY = (VEL_ALPHA * velocity.getYComponent()) + ((1 - VEL_ALPHA) * smoothedVelY);

        // --- A MÁGICA SEGURA: VELOCITY DEADBAND ---
        // Ignora ruídos e micro-vibrações menores que 1.5 in/s nas rodas de odometria.
        if (Math.abs(smoothedVelX) < 1.5) smoothedVelX = 0.0;
        if (Math.abs(smoothedVelY) < 1.5) smoothedVelY = 0.0;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

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

        turnController.setPIDF(ShooterConstants.ANGLE_KP, ShooterConstants.ANGLE_KI, ShooterConstants.ANGLE_KD, ShooterConstants.ANGLE_KF);
        double turnPower = turnController.calculate(error);

        double innerTolerance = Math.toRadians(ShooterConstants.ANGLE_TOLERANCE);
        double outerTolerance = innerTolerance + Math.toRadians(1.5);

        if (isAtTarget) {
            if (Math.abs(error) > outerTolerance) {
                isAtTarget = false;
            }
        } else {
            if (Math.abs(error) < innerTolerance) {
                isAtTarget = true;
            }
        }
        
        if (!isAtTarget) {
            turnPower += Math.copySign(ShooterConstants.ANGLE_KF, turnPower);
        }
        drivetrain.setAimLocked(isAtTarget);
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        follower.setTeleOpDrive(-strafe, forward, -turnPower, true);
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
        drivetrain.setAimLocked(false);
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}