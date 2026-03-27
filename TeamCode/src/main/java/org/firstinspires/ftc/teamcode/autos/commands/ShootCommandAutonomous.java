package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommandAutonomous extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final TelemetryManager telemetryM;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();

    // A máquina de estados nova aplicada no Autônomo!
    private enum SHOOT_STATES {
        Conveyor,
        Acceleration,
        Shooting,
        FollowThrough,
        Cooldown
    }

    private SHOOT_STATES state;
    private int shooterCounter;

    public ShootCommandAutonomous(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, int shoots) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.shooter = shooter;
        this.intake = intake;
        this.indexer = indexer;
        this.shooterCounter = shoots;
        addRequirements(shooter, indexer);
    }

    public ShootCommandAutonomous(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, LEDSubsystem led) {
        this(shooter, intake, indexer, 99); // Padrão do Auto: atirar tudo que tem na agulha
    }

    @Override
    public void initialize() {
        intake.run();
        intake.stopTrigger();
        state = SHOOT_STATES.Conveyor;
        timer.reset();
        indexer.setShootingState(true);
    }

    @Override
    public void execute() {

        switch (state) {
            case Conveyor:
                if (indexer.getExitSensor()) {
                    intake.stop();
                } else {
                    intake.run();
                }

                if (indexer.getExitSensor() || timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TO_SHOOT) {

                    // Mantido o isReady() do seu Auto para atirar mais rápido!
                    if (shooter.isReady()) {
                        state = SHOOT_STATES.Shooting;
                        shooter.anticipateShot();

                        // LIGA AQUI (Apenas 1 vez na transição!)
                        intake.runTrigger();
                        intake.run();

                        timer.reset();
                    } else {
                        state = SHOOT_STATES.Acceleration;
                    }
                }
                break;

            case Acceleration:
                if (indexer.getExitSensor()) {
                    intake.stop();
                } else {
                    intake.run();
                }

                if (shooter.isReady()) {
                    state = SHOOT_STATES.Shooting;
                    shooter.anticipateShot();

                    // LIGA AQUI (Apenas 1 vez na transição!)
                    intake.runTrigger();
                    intake.run();

                    timer.reset();
                }
                break;

            case Shooting:
                boolean pieceHasLeft = !indexer.getExitSensor();

                // Mantido o seu timeout de segurança de 1200ms do Autônomo
                if (pieceHasLeft || timer.milliseconds() > 1200) {
                    if (shooterCounter > 0) {
                        shooterCounter--;
                    }

                    state = SHOOT_STATES.FollowThrough;
                    timer.reset();
                }
                break;

            case FollowThrough:
                // O motor das rodas Compliant continua ligado na força máxima empurrando a argola.

                if (timer.milliseconds() > ShooterConstants.TRIGGER_FOLLOW_THROUGH_MS) {

                    if (shooterCounter > 0) {
                        state = SHOOT_STATES.Cooldown;

                        // DESLIGA AQUI (Apenas 1 vez!)
                        intake.stopTrigger();
                        intake.stop();
                        cooldownTimer.reset();
                    } else {
                        state = SHOOT_STATES.Conveyor;

                        // DESLIGA AQUI (Apenas 1 vez!)
                        intake.stopTrigger();
                        intake.stop();
                    }
                }
                break;

            case Cooldown:
                double time = cooldownTimer.milliseconds();

                // Mantido o seu cooldown fixo de 300ms do Autônomo
                if (time > 300) {
                    state = SHOOT_STATES.Conveyor;

                    if (indexer.getExitSensor()) intake.stop();
                    timer.reset();
                }
                break;
        }

        telemetryM.addData("Shoot State (Auto)", state);
        telemetryM.addData("Shots Left (Auto)", shooterCounter);
    }

    @Override
    public boolean isFinished() {
        return shooterCounter <= 0;
    }

    @Override
    public void end(boolean interrupted){
        intake.stopTrigger();
        intake.stop();
        indexer.setShootingState(false);
        telemetryM.addData("Shoot State (Auto)", "Finish");
    }
}