package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;

public class AimByPoseCommand extends CommandBase {

    private final Follower follower;
    private final PIDFController turnController;
    private final TelemetryManager telemetry;
    private final GamepadEx operator;
    private final double targetX;
    private final double targetY;
    boolean hasVibrated = false;

    public AimByPoseCommand(DrivetrainSubsystem drivetrain, double targetX, double targetY, TelemetryManager telemetry, GamepadEx operator) {

        this.follower = drivetrain.getFollower();
        this.targetX = targetX;
        this.targetY = targetY;
        this.telemetry = telemetry;
        this.operator = operator;
        turnController = new PIDFController(ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD,
                ShooterConstants.ANGLE_KF);


        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        turnController.reset();
        turnController.setSetPoint(0);
        turnController.setTolerance((Math.PI / 180.0) * ShooterConstants.ANGLE_TOLERANCE);  // ~0.2 graus
        hasVibrated = false;
    }

    @Override
    public void execute() {
        turnController.setPIDF(ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD,
                ShooterConstants.ANGLE_KF);

        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        double desired = Math.atan2( targetY - robotY,targetX - robotX);

        double error = angleDifference(desired, heading);

        double turnPower = turnController.calculate(error);
        turnPower += Math.copySign(ShooterConstants.ANGLE_KF, turnPower);

        if (turnController.atSetPoint()) {
            turnPower = 0;
            if (!hasVibrated && operator != null) {
                operator.gamepad.rumble(1, 1, 500); // Vibrate for 500ms
                hasVibrated = true; // Mark as vibrated
            }
        } else {
            hasVibrated = false;
        }
        // Limita rotação igual ao AlignToAprilTag
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        Log.d("AIMBYPOSE - Desired", String.valueOf(Math.toDegrees(desired)));
        Log.d("AIMBYPOSE - Turn Power", String.valueOf(turnPower));
        Log.d("AIMBYPOSE - At SetPoint", String.valueOf(turnController.atSetPoint()));
        Log.d("AIMBYPOSE - Heading", String.valueOf(Math.toDegrees(heading)));
        Log.d("AIMBYPOSE - Error", String.valueOf(Math.toDegrees(error)));

        // Apenas rotaciona o robô
        follower.setTeleOpDrive(0, 0, -turnPower, true);
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
