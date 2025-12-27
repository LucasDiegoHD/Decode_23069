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
public class AdjustShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;


    /**
     * Creates a new SpinShooterCommand.
     *
     * @param shooter The ShooterSubsystem to control.
     */
    public AdjustShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
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
        double rpm = ShooterConstants.RPM_N0 + ShooterConstants.RPM_N1 * distance;
        if ((distance > VisionConstants.LONGEST_DISTANCE || distance == 0)) {
            rpm = VisionConstants.LONGEST_RPM;

        }
        shooter.setTargetVelocity(rpm);
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
