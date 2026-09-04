package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * Condução field-centric do piloto: entradas ao quadrado, limitação de taxa na aceleração e
 * desaceleração, trava de rumo quando o piloto solta o giro, e escala por tensão da bateria.
 *
 * <p>É o comando contínuo do drivetrain. Roda com prioridade base e
 * {@link InterruptedBehavior#SUSPEND}: comandos de botão que reservam o drivetrain (alinhar ao
 * AprilTag, mira cinemática) o suspendem, e o escalonador o retoma sozinho quando eles terminam —
 * sem passar pelo {@code start()} de novo, então a suavização não dá solavanco na volta.
 */
public final class TeleOpDriveCommand {

    private TeleOpDriveCommand() {
    }

    private static final double MAX_ACCELERATION = 8.5;
    private static final double MAX_DECELERATION = 10.0;
    private static final double NOMINAL_VOLTAGE = 13.5;
    private static final double MAX_VOLTAGE_SCALE = 1.25;

    // --- Constantes do Drive Straight ---
    private static final double TURN_DEADBAND = 0.04;
    private static final double HEADING_KD = 0.1;
    private static final double MAX_COMP_PWR = 0.3;

    /** Estado que persiste entre iterações. Um por comando construído. */
    private static final class State {
        double currentMagnitude;
        double currentAngle;
        long lastTime;
        double lastHeading;
    }

    public static Command teleOpDrive(DrivetrainSubsystem drivetrain, Gamepad driverGamepad) {
        final State s = new State();
        final AllianceEnum alliance = DataStorage.alliance;

        return Command.build()
                .setStart(() -> {
                    drivetrain.getFollower().startTeleopDrive();
                    s.lastTime = System.currentTimeMillis();

                    double rawY = driverGamepad.left_stick_x;
                    double rawX = -driverGamepad.left_stick_y;

                    double targetX = rawX * Math.abs(rawX);
                    double targetY = rawY * Math.abs(rawY);

                    s.currentMagnitude = Math.hypot(targetX, targetY);
                    s.currentAngle = (s.currentMagnitude > 0.01) ? Math.atan2(targetY, targetX) : 0.0;

                    s.lastHeading = drivetrain.getFollower().getPose().getHeading();
                })
                .setExecute(() -> {
                    Pose p = drivetrain.getFollower().getPose();
                    double heading = p.getHeading();

                    double rawY = driverGamepad.left_stick_x;
                    double rawX = -driverGamepad.left_stick_y;
                    double rawTurn = -driverGamepad.right_stick_x;

                    double targetX = rawX * Math.abs(rawX);
                    double targetY = rawY * Math.abs(rawY);
                    double targetTurn = rawTurn * Math.abs(rawTurn);

                    long currentTime = System.currentTimeMillis();
                    double dt = Math.max((currentTime - s.lastTime) / 1000.0, 0.001);
                    s.lastTime = currentTime;

                    // --- Lógica de Translação (Intacta) ---
                    double targetMagnitude = Math.hypot(targetX, targetY);
                    double targetAngle = (targetMagnitude > 0.01)
                            ? Math.atan2(targetY, targetX)
                            : s.currentAngle;

                    double delta;
                    if (targetMagnitude >= s.currentMagnitude) {
                        delta = MAX_ACCELERATION * dt;
                    } else {
                        delta = MAX_DECELERATION * dt;
                    }

                    if (Math.abs(targetMagnitude - s.currentMagnitude) <= delta) {
                        s.currentMagnitude = targetMagnitude;
                    } else {
                        s.currentMagnitude += Math.copySign(delta, targetMagnitude - s.currentMagnitude);
                    }

                    if (targetMagnitude > 0.05) {
                        double angleDiff = targetAngle - s.currentAngle;
                        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
                        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
                        s.currentAngle += angleDiff * Math.min(1.0, s.currentMagnitude * 8.0 * dt);
                    }

                    double smoothX = s.currentMagnitude * Math.cos(s.currentAngle);
                    double smoothY = s.currentMagnitude * Math.sin(s.currentAngle);

                    double headingDelta = heading - s.lastHeading;
                    while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
                    while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;

                    double angularVelocity = headingDelta / dt;
                    s.lastHeading = heading;

                    double finalTurnPower;
                    if (Math.abs(rawTurn) > TURN_DEADBAND) {
                        finalTurnPower = targetTurn;
                    } else {
                        finalTurnPower = -HEADING_KD * angularVelocity;
                        finalTurnPower = Math.max(-MAX_COMP_PWR, Math.min(MAX_COMP_PWR, finalTurnPower));
                    }

                    // Rotação field-centric
                    double xField = smoothX * Math.cos(heading) - smoothY * Math.sin(heading);
                    double yField = smoothX * Math.sin(heading) + smoothY * Math.cos(heading);

                    if (alliance == AllianceEnum.Blue) {
                        xField = -xField;
                        yField = -yField;
                    }

                    double voltage = Math.max(drivetrain.getVoltage(), 10.0);
                    double voltageScale = Math.min(NOMINAL_VOLTAGE / voltage, MAX_VOLTAGE_SCALE);

                    drivetrain.getFollower().setTeleOpDrive(
                            xField * voltageScale,
                            -yField * voltageScale,
                            finalTurnPower * voltageScale,
                            true
                    );
                })
                .setDone(() -> false)
                .requiring(drivetrain)
                .setPriority(0)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }
}
