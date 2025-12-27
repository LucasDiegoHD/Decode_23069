package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * A command that drives the robot to a specified {@link Pose} using a single Bezier curve.
 * This command is designed for simple, point-to-point movements within an autonomous sequence.
 * It generates a {@link PathChain} consisting of a single {@link BezierLine} from the robot's
 * current pose to the target pose. The heading is linearly interpolated between the start and end poses.
 */
public class LeaveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Pose pose;

    /**
     * Constructs a new GoToPoseCommand.
     *
     * @param drivetrain The drivetrain subsystem required by this command.
     */
    public LeaveCommand(@NonNull DrivetrainSubsystem drivetrain, Pose p) {
        this.drivetrain = drivetrain;
        pose = p;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        PathChain path = drivetrain.getFollower()
                .pathBuilder()
                .addPath(
                        new BezierLine(drivetrain.getFollower().getPose(), pose)
                )
                .setLinearHeadingInterpolation(drivetrain.getFollower().getHeading(), pose.getHeading())
                .build();
        drivetrain.getFollower().followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.getFollower().isBusy();
    }
}

