package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterSubsystem;

/**
 * Comandos one-shot do atirador: velocidade dos volantes e posição do capô.
 *
 * <p>Reúne o que antes eram cinco classes ({@code SpinShooterCommand}, {@code AdjustHoodCommand},
 * {@code AdjustShooterCommand}, {@code AdjustHoodCommandAuto}, {@code AdjustShooterCommandAuto}).
 *
 * <p>As variantes de teleop e de autônomo <b>não</b> foram fundidas numa só: elas têm
 * comportamentos diferentes de propósito. O teleop cai para a geometria da pose quando a Limelight
 * não vê alvo e compensa a velocidade do robô em direção à meta; o autônomo usa valores fixos de
 * segurança. Fundir as duas mudaria o comportamento em quadra.
 */
public final class ShooterCommands {

    private ShooterCommands() {
    }

    /** Ações discretas de velocidade do atirador. */
    public enum Action {
        /** Acelera para um tiro longo. */
        LONG_SHOOT,
        /** Acelera para um tiro curto. */
        SHORT_SHOOT,
        /** Para os volantes. */
        STOP
    }

    private static final double RPM_FORWARD_MULT = -5.0;   // Tira pouco RPM atacando
    private static final double RPM_BACKWARD_MULT = 120.0; // Coloca mais RPM fugindo
    private static final double MAX_SAFE_RPM = 4500;
    private static final double MIN_SAFE_RPM = 1000.0;

    /** Fallback do autônomo quando não há leitura de distância confiável. */
    private static final double AUTO_FALLBACK_RPM = 2920;
    private static final double AUTO_FALLBACK_HOOD = 0.71;

    /** Altura relativa à meta usada no fallback geométrico, em polegadas. */
    private static final double HOOD_DELTA_Z = 28.5;
    private static final double RPM_DELTA_Z = 38.75;
    private static final double INCHES_PER_METER = 39.3701;

    /**
     * Aplica uma ação discreta de velocidade.
     */
    public static Command spin(ShooterSubsystem shooter, Action action) {
        return Commands.instant(() -> {
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
        }).requiring(shooter);
    }

    /**
     * Ajusta o capô pela distância (teleop). Sem alvo na Limelight, deriva a distância da pose.
     */
    public static Command adjustHood(ShooterSubsystem shooter, VisionSubsystem vision,
                                     DrivetrainSubsystem drivetrain, double targetX, double targetY) {
        return Commands.instant(() -> {
            double distance = vision.getDirectDistanceToTarget().orElse(0.0);

            if (distance <= 0) {
                distance = distanceFromPose(drivetrain, targetX, targetY, HOOD_DELTA_Z);
            }

            double hood = hoodPolynomial(distance);

            if (distance > VisionConstants.LONGEST_DISTANCE) {
                hood = VisionConstants.LONGEST_HOOD;
            }

            shooter.setLongShotMode(distance > VisionConstants.LONGEST_DISTANCE);
            shooter.setHoodPosition(hood);
        }).requiring(shooter);
    }

    /**
     * Ajusta a velocidade pela distância (teleop), compensando a velocidade do robô em direção
     * à meta: fugindo, acrescenta RPM; atacando, tira um pouco.
     */
    public static Command adjustShooter(ShooterSubsystem shooter, VisionSubsystem vision,
                                        DrivetrainSubsystem drivetrain, double targetX, double targetY) {
        return Commands.instant(() -> {
            double distance = vision.getDirectDistanceToTarget().orElse(0.0);

            Pose pose = drivetrain.getFollower().getPose();
            double dx = targetX - pose.getX();
            double dy = targetY - pose.getY();
            double groundDistance = Math.hypot(dx, dy);

            if (distance <= 0.1) {
                distance = Math.hypot(groundDistance, RPM_DELTA_Z) / INCHES_PER_METER;
            }

            double baseRpm = rpmPolynomial(distance);

            Vector velocity = drivetrain.getFollower().getVelocity();
            double velTowardsGoal = 0.0;

            if (groundDistance > 1.0) {
                double dirX = dx / groundDistance;
                double dirY = dy / groundDistance;
                velTowardsGoal = (velocity.getXComponent() * dirX) + (velocity.getYComponent() * dirY);
            }

            double rpmAdjustment = velTowardsGoal > 0
                    ? velTowardsGoal * RPM_FORWARD_MULT
                    : velTowardsGoal * RPM_BACKWARD_MULT;

            double finalRpm = baseRpm - rpmAdjustment;

            if (distance > VisionConstants.LONGEST_DISTANCE) {
                finalRpm = VisionConstants.LONGEST_RPM;
            }

            finalRpm = Math.max(MIN_SAFE_RPM, Math.min(finalRpm, MAX_SAFE_RPM));
            shooter.setTargetVelocity(finalRpm);
        }).requiring(shooter);
    }

    /**
     * Ajusta o capô pela distância (autônomo). Sem fallback geométrico: distância zero ou acima
     * do alcance cai no valor fixo de segurança.
     */
    public static Command adjustHoodAuto(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Commands.instant(() -> {
            double distance = vision.getDirectDistanceToTarget().orElse(0.0);

            double hood = hoodPolynomial(distance);

            if (distance > VisionConstants.LONGEST_DISTANCE || distance == 0) {
                hood = AUTO_FALLBACK_HOOD;
            }

            shooter.setLongShotMode(distance > VisionConstants.LONGEST_DISTANCE);
            shooter.setHoodPosition(hood);
        }).requiring(shooter);
    }

    /**
     * Ajusta a velocidade pela distância (autônomo). Sem compensação de movimento.
     */
    public static Command adjustShooterAuto(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Commands.instant(() -> {
            double distance = vision.getDirectDistanceToTarget().orElse(0.0);

            double rpm = rpmPolynomial(distance);

            if (distance > VisionConstants.LONGEST_DISTANCE || distance == 0) {
                rpm = AUTO_FALLBACK_RPM;
            }

            shooter.setTargetVelocity(rpm);
        }).requiring(shooter);
    }

    /**
     * Prepara o atirador para um tiro de autônomo: velocidade e depois capô.
     */
    public static Command alignAndAdjustAuto(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Groups.sequential(
                adjustShooterAuto(shooter, vision),
                adjustHoodAuto(shooter, vision)
        ).requiring(shooter);
    }

    /** Polinômio distância → posição do capô, em forma de Horner. */
    private static double hoodPolynomial(double distance) {
        return ShooterConstants.HOOD_N0
                + distance * (ShooterConstants.HOOD_N1
                + distance * (ShooterConstants.HOOD_N2
                + distance * ShooterConstants.HOOD_N3));
    }

    /** Polinômio distância → RPM, em forma de Horner. */
    private static double rpmPolynomial(double distance) {
        return ShooterConstants.RPM_N0
                + distance * (ShooterConstants.RPM_N1
                + distance * ShooterConstants.RPM_N2);
    }

    /** Distância em metros até (targetX, targetY), corrigida pela altura da meta. */
    private static double distanceFromPose(DrivetrainSubsystem drivetrain,
                                           double targetX, double targetY, double deltaZ) {
        Pose pose = drivetrain.getFollower().getPose();
        double groundDistance = Math.hypot(targetX - pose.getX(), targetY - pose.getY());
        return Math.hypot(groundDistance, deltaZ) / INCHES_PER_METER;
    }
}
