package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.Arrays;
import java.util.List;

public class GoToPoseCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final List<Pose> waypoints;
    private final boolean holdEnd;
    private PathConstraints constraints;

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, Pose targetPose) {
        this(drivetrain, true, targetPose);
    }

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, boolean holdEnd, Pose targetPose) {
        this(drivetrain, holdEnd, new Pose[]{targetPose});
    }

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, boolean holdEnd, Pose... poses) {
        this.drivetrain = drivetrain;
        this.holdEnd = holdEnd;
        this.waypoints = Arrays.asList(poses);
        this.constraints = Constants.pathConstraints;
        addRequirements(drivetrain);
    }

    public GoToPoseCommand setConstraints(PathConstraints customConstraints) {
        this.constraints = customConstraints;
        return this;
    }

    @Override
    public void initialize() {
        if (waypoints.isEmpty()) return;

        Pose startPose = drivetrain.getFollower().getPose();
        PathBuilder builder = drivetrain.getFollower().pathBuilder();

        builder.addPath(new BezierLine(startPose, waypoints.get(0)));
        builder.setLinearHeadingInterpolation(startPose.getHeading(), waypoints.get(0).getHeading());
        applyConstraints(builder);

        for (int i = 1; i < waypoints.size(); i++) {
            Pose previous = waypoints.get(i - 1);
            Pose current = waypoints.get(i);

            builder.addPath(new BezierLine(previous, current));
            builder.setLinearHeadingInterpolation(previous.getHeading(), current.getHeading());
            applyConstraints(builder);
        }

        PathChain chain = builder.build();

        drivetrain.getFollower().followPath(chain, holdEnd);
    }

    private void applyConstraints(PathBuilder builder) {
        if (constraints != null) {
            builder.setConstraints(constraints);
        }
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.getFollower().isBusy();
    }
}