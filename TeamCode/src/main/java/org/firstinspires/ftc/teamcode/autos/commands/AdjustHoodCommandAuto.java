package org.firstinspires.ftc.teamcode.autos.commands;

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
public class AdjustHoodCommandAuto extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    /**
     * Creates a new AdjustHoodCommand.
     *
     * @param shooter The ShooterSubsystem to control.
     */
    public AdjustHoodCommandAuto(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
    }

    /**
     * Called when the command is initially scheduled. Executes the specified hood action.
     */
    @Override
    public void initialize() {
        double distance = vision.getDirectDistanceToTarget().orElse((double) 0);

        double hood = ShooterConstants.HOOD_N0 + ShooterConstants.HOOD_N1 * distance
                + ShooterConstants.HOOD_N2 * Math.pow(distance, 2) + ShooterConstants.HOOD_N3 * Math.pow(distance, 3);

        if (distance > VisionConstants.LONGEST_DISTANCE) {
            hood = VisionConstants.LONGEST_HOOD;
        }

        if (distance == 0){
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