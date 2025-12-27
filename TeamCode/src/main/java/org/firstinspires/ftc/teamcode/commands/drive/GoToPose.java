// Ficheiro: commands/drivetrain/GoToPose.java
package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * A command to move the robot to a specified target Pose using a simple Bezier curve.
 */
public class GoToPose extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Pose targetPose;

    /**
     * Creates a new GoToPoseCommand.
     *
     * @param drivetrain The DrivetrainSubsystem to use for movement.
     * @param targetPose The target Pose to move the robot to.
     */
    public GoToPose(DrivetrainSubsystem drivetrain, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        addRequirements(drivetrain);
    }

    /**
     * Called when the command is initially scheduled. It creates a path from the robot's current
     * pose to the target pose and starts following it.
     */
    @Override
    public void initialize() {
        Pose startPose = drivetrain.getFollower().getPose();

        // Create a simple path (a Bezier line) from the current pose to the target.
        PathChain pathToTarget = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();

        // Command the drivetrain to follow the generated path.
        drivetrain.getFollower().followPath(pathToTarget);
    }

    /**
     * Returns true when the command should end.
     *
     * @return True when the robot is no longer busy following the path.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}