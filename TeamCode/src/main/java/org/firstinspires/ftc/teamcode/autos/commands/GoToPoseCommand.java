package org.firstinspires.ftc.teamcode.autos.commands;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Constroi e segue um caminho da pose atual ate um a tres waypoints.
 *
 * <p>E um builder: configure com os metodos fluentes e chame {@link #toCommand()} no fim. O
 * caminho so e montado quando o comando inicia, para partir da pose real do robo naquele momento.
 */
public class GoToPoseCommand {
    private final DrivetrainSubsystem drivetrain;
    private final List<Pose> waypoints;
    private final boolean holdEnd;
    private PathConstraints constraints;
    private double pathMaxPower = 1.0;
    private enum DecelerationMode { DEFAULT, GLOBAL, NONE }
    private DecelerationMode decelerationMode = DecelerationMode.DEFAULT;

    // Modos de Ângulo (Heading)
    public enum HeadingMode { LINEAR, TANGENT, CONSTANT }
    private HeadingMode headingMode = HeadingMode.LINEAR;
    private double customConstantHeading = Double.NaN;

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
    }

    // Builder para Constraints
    public GoToPoseCommand setConstraints(PathConstraints customConstraints) {
        this.constraints = customConstraints;
        return this;
    }

    // Builder para Potência (Útil se não quiser mexer nas constraints)
    public GoToPoseCommand withMaxPower(double maxPower) {
        this.pathMaxPower = maxPower;
        return this;
    }

    // Builders para Desaceleração
    public GoToPoseCommand withNoDeceleration() {
        this.decelerationMode = DecelerationMode.NONE;
        return this;
    }

    public GoToPoseCommand withGlobalDeceleration() {
        this.decelerationMode = DecelerationMode.GLOBAL;
        return this;
    }

    // Builders para Modos de Ângulo
    public GoToPoseCommand withTangentHeading() {
        this.headingMode = HeadingMode.TANGENT;
        return this;
    }

    /**
     * Trava o ângulo. Automaticamente usa o ângulo do PRIMEIRO waypoint passado.
     */
    public GoToPoseCommand withConstantHeading() {
        this.headingMode = HeadingMode.CONSTANT;
        this.customConstantHeading = Double.NaN;
        return this;
    }
    private double exitTolerance = -1.0;

    // Builder para definir o raio de chegada
    public GoToPoseCommand withExitTolerance(double inches) {
        this.exitTolerance = inches;
        return this;
    }

    /** Fecha o builder e devolve o comando agendavel. */
    public Command toCommand() {
        return Command.build()
                .setStart(this::followPath)
                .setDone(this::isDone)
                .setEnd(endCondition -> drivetrain.getFollower().setMaxPower(1.0))
                .requiring(drivetrain);
    }

    private void followPath() {
        if (waypoints.isEmpty()) return;

        Pose startPose = drivetrain.getFollower().getPose();
        PathBuilder builder = drivetrain.getFollower().pathBuilder();

        int size = waypoints.size();

        if (size == 1) {
            builder.addPath(new BezierLine(startPose, waypoints.get(0)));
        } else if (size == 2) {
            builder.addPath(new BezierCurve(startPose, waypoints.get(0), waypoints.get(1)));
        } else if (size == 3) {
            builder.addPath(new BezierCurve(startPose, waypoints.get(0), waypoints.get(1), waypoints.get(2)));
        }

        Pose lastPose = waypoints.get(size - 1);
        switch (headingMode) {
            case TANGENT:
                builder.setTangentHeadingInterpolation();
                break;
            case CONSTANT:
                double targetAngle = Double.isNaN(customConstantHeading)
                        ? waypoints.get(0).getHeading()
                        : customConstantHeading;
                builder.setConstantHeadingInterpolation(targetAngle);
                break;
            case LINEAR:
            default:
                builder.setLinearHeadingInterpolation(startPose.getHeading(), lastPose.getHeading());
                break;
        }

        applyConstraints(builder);
        PathChain chain = builder.build();

        switch (decelerationMode) {
            case NONE:
                chain.setDecelerationType(PathChain.DecelerationType.NONE);
                break;
            case GLOBAL:
                chain.setDecelerationType(PathChain.DecelerationType.GLOBAL);
                break;
            case DEFAULT:
            default:
                break;
        }

        drivetrain.getFollower().setMaxPower(this.pathMaxPower);
        drivetrain.getFollower().followPath(chain, holdEnd);
    }

    private void applyConstraints(PathBuilder builder) {
        if (constraints != null) {
            builder.setConstraints(constraints);
        }
    }

    private boolean isDone() {
        if (exitTolerance > 0 && !waypoints.isEmpty()) {
            Pose currentPose = drivetrain.getFollower().getPose();
            Pose targetPose = waypoints.get(waypoints.size() - 1);

            double distance = Math.hypot(
                    currentPose.getX() - targetPose.getX(),
                    currentPose.getY() - targetPose.getY()
            );

            if (distance <= exitTolerance) {
                return true;
            }
        }

        return !drivetrain.getFollower().isBusy();
    }
}