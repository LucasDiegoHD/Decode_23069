package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * Condução do piloto com mira automática: o piloto controla a translação, o comando controla o
 * rumo, apontando para onde a meta <i>estará</i> quando o artefato chegar.
 *
 * <p>A predição soma tempo de voo, latência do sistema e a velocidade tangencial gerada pela
 * própria rotação do robô, e ainda aplica um feedforward proporcional à velocidade lateral.
 * O trava/destrava usa histerese (tolerância interna e externa) para não oscilar no limiar.
 *
 * <p>Reserva o drivetrain com prioridade 1, acima do comando contínuo de condução.
 */
public final class KinematicAimDriveCommand {

    private KinematicAimDriveCommand() {
    }

    private static final double ARTIFACT_VELOCITY_INCHES_PER_SEC = 1000.0;
    private static final double SYSTEM_LATENCY_SECONDS = 0.4;
    private static final double FEEDFORWARD_DEAD_ZONE = Math.toRadians(2.0);
    private static final double SHOOTER_RADIUS_INCHES = 4.0;
    private static final double VEL_ALPHA = 0.8;

    /** Estado que persiste entre iterações. Um por comando construído. */
    private static final class State {
        boolean isAtTarget;
        double smoothedVelX;
        double smoothedVelY;
    }

    public static Command kinematicAimDrive(DrivetrainSubsystem drivetrain, Gamepad driver,
                                            double targetX, double targetY) {
        final Follower follower = drivetrain.getFollower();
        final State s = new State();
        final PIDFController turnController = new PIDFController(new PIDFCoefficients(
                ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD,
                0));

        return Command.build()
                .setStart(() -> {
                    follower.startTeleopDrive();
                    turnController.reset();

                    s.smoothedVelX = follower.getVelocity().getXComponent();
                    s.smoothedVelY = follower.getVelocity().getYComponent();
                    s.isAtTarget = false;
                })
                .setExecute(() -> {
                    Pose pose = follower.getPose();
                    Vector velocity = follower.getVelocity();
                    double heading = pose.getHeading();

                    double rawY = driver.left_stick_x;
                    double rawX = -driver.left_stick_y;

                    double targetYInput = rawY * Math.abs(rawY);
                    double targetXInput = rawX * Math.abs(rawX);

                    double xField = targetXInput * Math.cos(heading) - targetYInput * Math.sin(heading);
                    double yField = targetXInput * Math.sin(heading) + targetYInput * Math.cos(heading);

                    if (DataStorage.alliance == AllianceEnum.Blue) {
                        xField = -xField;
                        yField = -yField;
                    }

                    s.smoothedVelX = (VEL_ALPHA * velocity.getXComponent()) + ((1 - VEL_ALPHA) * s.smoothedVelX);
                    s.smoothedVelY = (VEL_ALPHA * velocity.getYComponent()) + ((1 - VEL_ALPHA) * s.smoothedVelY);

                    double velMagnitude = Math.hypot(s.smoothedVelX, s.smoothedVelY);
                    if (velMagnitude < 2.0) {
                        s.smoothedVelX = 0.0;
                        s.smoothedVelY = 0.0;
                    }

                    double robotX = pose.getX();
                    double robotY = pose.getY();

                    double diffX = targetX - robotX;
                    double diffY = targetY - robotY;
                    double distanceToTarget = Math.max(Math.hypot(diffX, diffY), 1.0);

                    double targetDirX = diffX / distanceToTarget;
                    double targetDirY = diffY / distanceToTarget;
                    double velTowardsGoal = (s.smoothedVelX * targetDirX) + (s.smoothedVelY * targetDirY);

                    double effectiveArtifactVelocity =
                            Math.max(ARTIFACT_VELOCITY_INCHES_PER_SEC + velTowardsGoal, 100.0);
                    double timeOfFlight = distanceToTarget / effectiveArtifactVelocity;
                    double totalPredictionTime = timeOfFlight + SYSTEM_LATENCY_SECONDS;

                    double angularVel = follower.getAngularVelocity();
                    double tangentialVelMagnitude = angularVel * SHOOTER_RADIUS_INCHES;
                    double tangentialVelX = tangentialVelMagnitude * -targetDirY;
                    double tangentialVelY = tangentialVelMagnitude * targetDirX;

                    double virtualX = targetX - (s.smoothedVelX * totalPredictionTime)
                            - (tangentialVelX * totalPredictionTime);
                    double virtualY = targetY - (s.smoothedVelY * totalPredictionTime)
                            - (tangentialVelY * totalPredictionTime);

                    double lateralVel = (s.smoothedVelX * -targetDirY) + (s.smoothedVelY * targetDirX);
                    double omegaFeedforward = (lateralVel / distanceToTarget) * ShooterConstants.K_OMEGA;

                    double desiredAngle = Math.atan2(virtualY - robotY, virtualX - robotX);
                    double error = angleDifference(desiredAngle, heading);

                    // O alvo é erro zero, então o erro do controlador é -error.
                    turnController.updateError(-error);
                    double turnPower = turnController.run();

                    double innerTolerance = Math.toRadians(ShooterConstants.ANGLE_TOLERANCE);
                    double outerTolerance = innerTolerance + Math.toRadians(0.4);

                    if (s.isAtTarget) {
                        if (Math.abs(error) > outerTolerance) s.isAtTarget = false;
                    } else {
                        if (Math.abs(error) < innerTolerance) s.isAtTarget = true;
                    }

                    if (!s.isAtTarget && Math.abs(error) > FEEDFORWARD_DEAD_ZONE) {
                        turnPower += Math.copySign(ShooterConstants.ANGLE_KF, turnPower);
                    } else if (s.isAtTarget) {
                        turnPower = 0.0;
                    }

                    turnPower += omegaFeedforward;
                    turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

                    follower.setTeleOpDrive(
                            xField,
                            -yField,
                            -turnPower,
                            true
                    );
                })
                .setDone(() -> false)
                .requiring(drivetrain)
                .setPriority(1);
    }

    private static double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
