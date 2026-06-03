package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command to automatically align the robot to an AprilTag target.
 * Logic:
 * 1. Locks X and Y movement (Robot stays in place).
 * 2. Rotates to align Tx to 0.
 * 3. Rumbles the operator controller once when alignment is reached.
 * 4. Re-arms the rumble if the robot loses alignment and realigns.
 */
public class AlignToAprilTagCommand extends CommandBase {

    private final Follower follower;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry;
    private final PIDFController turnController;
    private int IsAprilTagNotSeemCounter = 0;
    private static final int ApriltagNotSeemMaximumCounter = 20;
    private final GamepadEx operator;
    boolean hasVibrated = false;

    public AlignToAprilTagCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TelemetryManager telemetry, GamepadEx operator) {
        this.follower = drivetrain.getFollower();
        this.vision = vision;
        this.telemetry = telemetry;
        this.operator = operator;
        this.turnController = new PIDFController(VisionConstants.TURN_KP, VisionConstants.TURN_KI, VisionConstants.TURN_KD, VisionConstants.TURN_KF);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        turnController.reset();
        // Explicitly set the target to 0 (center of the image)
        turnController.setSetPoint(0);

        // If it oscillates too much, increase this slightly.
        turnController.setTolerance(0.1);
        hasVibrated = false;
    }

    @Override
    public void execute() {

        turnController.setPIDF(
                VisionConstants.TURN_KP,
                VisionConstants.TURN_KI,
                VisionConstants.TURN_KD,
                VisionConstants.TURN_KF
        );

        if (!vision.hasTarget()) {

            follower.setTeleOpDrive(0, 0, 0, true);
            telemetry.debug("No AprilTag detected");

            hasVibrated = false;

            IsAprilTagNotSeemCounter++;
        } else {
            IsAprilTagNotSeemCounter = 0;

            // PID Calculation
            double currentTx = vision.getTargetTx().orElse(0.0);
            double turnPower = turnController.calculate(currentTx);

            turnPower = Math.max(-0.8, Math.min(0.8, turnPower));


            if (turnController.atSetPoint()) {
                if (!hasVibrated && operator != null) {
                    operator.gamepad.rumble(1, 1, 500); // Vibrate for 500ms
                    hasVibrated = true; // Mark as vibrated
                }
            } else {

                hasVibrated = false;
            }

            telemetry.debug("Align TX", currentTx);
            telemetry.debug("Turn Power", turnPower);
            telemetry.debug("At SetPoint", turnController.atSetPoint());

            follower.setTeleOpDrive(0, 0, turnPower, true);
        }
    }


    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
        telemetry.debug("Alignment finished.");
        telemetry.update();
    }
    @Override
    public boolean isFinished() {
        if (vision.hasTarget() && turnController.atSetPoint()) {
            return true;
        }

        if (IsAprilTagNotSeemCounter >= ApriltagNotSeemMaximumCounter) {
            return true;
        }
        return false;
    }
}