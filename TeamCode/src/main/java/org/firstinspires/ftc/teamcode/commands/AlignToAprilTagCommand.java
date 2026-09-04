package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * Alinha o robô a um AprilTag girando no lugar até zerar o {@code tx} da Limelight.
 *
 * <p>Trava translação (X e Y ficam em zero), gira por PIDF, e vibra o controle do operador uma
 * vez ao travar no alvo — rearmando a vibração se o alinhamento se perder.
 *
 * <p>Reserva o drivetrain com prioridade 1, acima do comando contínuo de condução: agendá-lo
 * suspende a condução manual, que o escalonador retoma sozinha quando este termina.
 */
public final class AlignToAprilTagCommand {

    private AlignToAprilTagCommand() {
    }

    private static final int APRILTAG_NOT_SEEN_MAXIMUM_COUNTER = 20;

    /** Tolerância de {@code tx} para considerar alinhado, em graus. */
    private static final double TX_TOLERANCE = 0.1;

    /**
     * Verdadeiro enquanto um alinhamento está em execução.
     *
     * <p>O Ivy não expõe o comando que detém um recurso (não há {@code getCurrentCommand()}), e o
     * laço periódico de relocalização precisa saber se o robô está mirando para não relocalizar no
     * meio de um tiro. Esta flag substitui o antigo
     * {@code drivetrain.getCurrentCommand() instanceof AlignToAprilTagCommand}.
     */
    private static volatile boolean aligning = false;

    /** Se há um alinhamento em andamento. */
    public static boolean isAligning() {
        return aligning;
    }

    public static Command alignToAprilTag(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
                                          TelemetryManager telemetry, Gamepad operator) {
        final Follower follower = drivetrain.getFollower();
        final PIDFController turnController = new PIDFController(new PIDFCoefficients(
                VisionConstants.TURN_KP,
                VisionConstants.TURN_KI,
                VisionConstants.TURN_KD,
                VisionConstants.TURN_KF));

        final State s = new State();

        return Command.build()
                .setStart(() -> {
                    turnController.reset();
                    s.hasVibrated = false;
                    s.notSeenCounter = 0;
                    s.atSetPoint = false;
                    aligning = true;
                })
                .setExecute(() -> {
                    turnController.setCoefficients(new PIDFCoefficients(
                            VisionConstants.TURN_KP,
                            VisionConstants.TURN_KI,
                            VisionConstants.TURN_KD,
                            VisionConstants.TURN_KF));

                    if (!vision.hasTarget()) {
                        follower.setTeleOpDrive(0, 0, 0, true);
                        telemetry.debug("No AprilTag detected");

                        s.hasVibrated = false;
                        s.atSetPoint = false;
                        s.notSeenCounter++;
                    } else {
                        s.notSeenCounter = 0;

                        double currentTx = vision.getTargetTx().orElse(0.0);

                        // O alvo é tx = 0, então o erro é -tx.
                        turnController.updateError(-currentTx);
                        double turnPower = turnController.run();

                        turnPower = Math.max(-0.8, Math.min(0.8, turnPower));

                        s.atSetPoint = Math.abs(currentTx) < TX_TOLERANCE;

                        if (s.atSetPoint) {
                            if (!s.hasVibrated && operator != null) {
                                operator.rumble(1, 1, 500); // Vibrate for 500ms
                                s.hasVibrated = true;
                            }
                        } else {
                            s.hasVibrated = false;
                        }

                        telemetry.debug("Align TX", currentTx);
                        telemetry.debug("Turn Power", turnPower);
                        telemetry.debug("At SetPoint", s.atSetPoint);

                        follower.setTeleOpDrive(0, 0, turnPower, true);
                    }
                })
                .setDone(() -> (vision.hasTarget() && s.atSetPoint)
                        || s.notSeenCounter >= APRILTAG_NOT_SEEN_MAXIMUM_COUNTER)
                .setEnd(endCondition -> {
                    aligning = false;
                    follower.setTeleOpDrive(0, 0, 0, true);
                    telemetry.debug("Alignment finished.");
                    telemetry.update();
                })
                .requiring(drivetrain)
                .setPriority(1);
    }

    /** Estado que persiste entre iterações. Um por comando construído. */
    private static final class State {
        int notSeenCounter;
        boolean hasVibrated;
        boolean atSetPoint;
    }
}
