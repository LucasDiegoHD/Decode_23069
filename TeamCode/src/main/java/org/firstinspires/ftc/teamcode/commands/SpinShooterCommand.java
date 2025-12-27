package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

/**
 * A command to spin the shooter motors to a specific velocity or stop them.
 * This is an instant command (finishes immediately) that just sets the state of the shooter.
 */
public class SpinShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Action action;

    /**
     * Defines the possible actions for the shooter.
     */
    public enum Action {
        /**
         * Spin up for a long-distance shot.
         */
        LONG_SHOOT,
        /** Spin up for a short-distance shot. */
        SHORT_SHOOT,
        /** Stop the shooter motors. */
        STOP
    }

    /**
     * Creates a new SpinShooterCommand.
     *
     * @param shooter The ShooterSubsystem to control.
     * @param action  The action to perform (LONG_SHOOT, SHORT_SHOOT, or STOP).
     */
    public SpinShooterCommand(ShooterSubsystem shooter, Action action) {
        this.shooter = shooter;
        this.action = action;

        addRequirements(shooter);
    }

    /**
     * Called when the command is initially scheduled. Executes the specified shooter action.
     */
    @Override
    public void initialize() {
        switch (action) {
            case LONG_SHOOT:
                shooter.setTargetVelocity(ShooterConstants.TARGET_VELOCITY_LONG);
                break;
            case SHORT_SHOOT:
                shooter.setTargetVelocity(ShooterConstants.TARGET_VELOCITY_SHORT);
                break;
            case STOP:
                shooter.stop();
                break;
        }
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
