package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.HashMap;
import java.util.Map;

public class GoToPoseCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Pose targetPose;
    private double cruiseSpeed = 1.0;
    private PathConstraints constraints;
    private final Map<Double, Runnable> callbacks = new HashMap<>();

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        addRequirements(drivetrain);
    }

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, Pose targetPose, double speed, PathConstraints constraints) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.cruiseSpeed = speed;
        this.constraints = constraints;
        addRequirements(drivetrain);
    }

    public GoToPoseCommand addCallback(double t, Runnable action) {
        callbacks.put(t, action);
        return this;
    }

    @Override
    public void initialize() {
        Pose startPose = drivetrain.getFollower().getPose();

        drivetrain.getFollower().setMaxPower(cruiseSpeed);

        PathBuilder builder = drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading());

        if (constraints != null) {
            builder.setTValueConstraint(constraints.getTValueConstraint());
            builder.setTimeoutConstraint(constraints.getTimeoutConstraint());
        }

        for (Map.Entry<Double, Runnable> entry : callbacks.entrySet()) {
            builder.addParametricCallback(entry.getKey(), entry.getValue());
        }

        PathChain path = builder.build();
        drivetrain.getFollower().followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !drivetrain.getFollower().isBusy();
    }
}