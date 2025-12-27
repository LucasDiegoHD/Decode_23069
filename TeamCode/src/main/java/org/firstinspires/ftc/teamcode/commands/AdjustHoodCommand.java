package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

/**
 * A command to spin the shooter motors to a specific velocity or stop them.
 * This is an instant command (finishes immediately) that just sets the state of the shooter.
 */
public class AdjustHoodCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;


    /**
     * Creates a new SpinShooterCommand.
     *
     * @param shooter The ShooterSubsystem to control.
     */
    public AdjustHoodCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;

        addRequirements(shooter, vision);
    }

    /**
     * Called when the command is initially scheduled. Executes the specified shooter action.
     */
    @Override
    public void initialize() {
        double distance = vision.getDirectDistanceToTarget().orElse((double) 0);
        double hood = ShooterConstants.HOOD_N0 + ShooterConstants.HOOD_N1 * distance
                + ShooterConstants.HOOD_N2 * Math.pow(distance, 2) + ShooterConstants.HOOD_N3 * Math.pow(distance, 3);
        if (distance > VisionConstants.LONGEST_DISTANCE || distance == 0) {
            hood = VisionConstants.LONGEST_HOOD;
        }

        // Define "Long Shot" as more than 65% of the Longest Distance
        // Or you can use a fixed value like: distance > 1.5
        boolean longShotMode = distance > (VisionConstants.LONGEST_DISTANCE * 0.65);
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
