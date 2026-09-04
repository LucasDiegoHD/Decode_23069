package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterSubsystem;

import java.util.function.IntSupplier;

/**
 * Máquina de estados de alimentação e disparo, em cinco passos:
 * esteira → aceleração → disparo → acompanhamento → resfriamento.
 *
 * <p>Unifica o antigo {@code ShootCommand} (teleop) e {@code ShootCommandAutonomous}: as duas
 * máquinas eram funcionalmente idênticas, diferindo só no rótulo de telemetria, no rumble do
 * piloto e em o autônomo receber a contagem por {@link IntSupplier}. A contagem é lida no
 * {@code start()}, então um supplier permite decidi-la no momento do disparo.
 */
public final class ShootCommand {

    private ShootCommand() {
    }

    private enum ShootState {
        Conveyor,
        Acceleration,
        Shooting,
        FollowThrough,
        Cooldown
    }

    /** Estado que persiste entre iterações. Um por comando construído. */
    private static final class State {
        final ElapsedTime timer = new ElapsedTime();
        final ElapsedTime cooldownTimer = new ElapsedTime();
        ShootState state;
        int shooterCounter;
    }

    /** Dispara continuamente até ser interrompido (contagem efetivamente infinita). */
    public static Command shoot(ShooterSubsystem shooter, IntakeSubsystem intake,
                                IndexerSubsystem indexer, Gamepad driver) {
        return shoot(shooter, intake, indexer, () -> 500, driver);
    }

    /** Dispara uma quantidade fixa de peças. */
    public static Command shoot(ShooterSubsystem shooter, IntakeSubsystem intake,
                                IndexerSubsystem indexer, int shootCount) {
        return shoot(shooter, intake, indexer, () -> shootCount, null);
    }

    /**
     * Dispara a quantidade que o supplier informar no momento do início.
     *
     * @param driver controle a vibrar ao iniciar; {@code null} no autônomo.
     */
    public static Command shoot(ShooterSubsystem shooter, IntakeSubsystem intake,
                                IndexerSubsystem indexer, IntSupplier shootCount, Gamepad driver) {
        final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        final State s = new State();

        return Command.build()
                .setStart(() -> {
                    s.shooterCounter = shootCount.getAsInt();
                    intake.run();
                    intake.stopTrigger();
                    s.state = ShootState.Conveyor;
                    s.timer.reset();
                    indexer.setShootingState(true);
                    if (driver != null) {
                        driver.rumble(150);
                    }
                })
                .setExecute(() -> {
                    switch (s.state) {
                        case Conveyor:
                            if (indexer.getExitSensor()) {
                                intake.stop();
                            } else {
                                intake.run();
                            }

                            if (indexer.getExitSensor()
                                    || s.timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TO_SHOOT) {
                                if (shooter.getShooterAtTarget()) {
                                    s.state = ShootState.Shooting;
                                    shooter.anticipateShot();
                                    intake.runTrigger();
                                    intake.run();

                                    s.timer.reset();
                                } else {
                                    s.state = ShootState.Acceleration;
                                }
                            }
                            break;

                        case Acceleration:
                            if (indexer.getExitSensor()) {
                                intake.stop();
                            } else {
                                intake.run();
                            }

                            if (shooter.getShooterAtTarget()) {
                                s.state = ShootState.Shooting;
                                shooter.anticipateShot();
                                intake.runTrigger();
                                intake.run();

                                s.timer.reset();
                            }
                            break;

                        case Shooting:
                            boolean pieceHasLeft = !indexer.getExitSensor();

                            if (pieceHasLeft
                                    || s.timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TRIGGERING) {
                                if (s.shooterCounter > 0) {
                                    s.shooterCounter--;
                                }

                                s.state = ShootState.FollowThrough;
                                s.timer.reset();
                            }
                            break;

                        case FollowThrough:
                            if (s.timer.milliseconds() > ShooterConstants.TRIGGER_FOLLOW_THROUGH_MS) {
                                intake.stopTrigger();
                                intake.stop();

                                if (s.shooterCounter > 0) {
                                    s.state = ShootState.Cooldown;
                                    s.cooldownTimer.reset();
                                } else {
                                    s.state = ShootState.Conveyor;
                                }
                            }
                            break;

                        case Cooldown:
                            if (s.cooldownTimer.milliseconds() > ShooterConstants.DELAY_BETWEEN_SHOTS_MS) {
                                s.state = ShootState.Conveyor;

                                if (indexer.getExitSensor()) intake.stop();
                                s.timer.reset();
                            }
                            break;
                    }

                    telemetryM.addData("Shoot State", s.state);
                    telemetryM.addData("Shots Left", s.shooterCounter);
                })
                .setDone(() -> s.shooterCounter <= 0)
                .setEnd(endCondition -> {
                    intake.stopTrigger();
                    intake.stop();
                    indexer.setShootingState(false);
                })
                .requiring(shooter, indexer);
    }
}
