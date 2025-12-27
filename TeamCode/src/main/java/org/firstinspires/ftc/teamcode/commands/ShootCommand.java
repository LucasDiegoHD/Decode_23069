package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

/**
 * The ShootCommand orchestrates the process of shooting game pieces.
 * It manages a state machine to control the intake, shooter acceleration, and triggering.
 */
public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Defines the states of the shooting process.
     */
    private enum SHOOT_STATES{
        /**
         * The state where the conveyor is running to feed a piece.
         */
        Conveyor,

        Acceleration,
        /**
         * The state after a piece is shot, waiting for the shooter to regain speed.
         */
        Shooting
    }

    private SHOOT_STATES state;
    private int shooterCounter;
    private final TelemetryManager telemetryM;
    private final IndexerSubsystem indexer;
    private final boolean lastSensor = false;

    /**
     * Constructs a new ShootCommand.
     * @param shooter The ShooterSubsystem to use.
     * @param intake The IntakeSubsystem to use.
     * @param shoots The number of pieces to shoot. Use a high number for continuous shooting.
     */
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, int shoots) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.indexer = indexer;
        this.shooterCounter = shoots;
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter, indexer);
    }

    /**
     * Constructs a new ShootCommand for continuous shooting.
     * @param shooter The ShooterSubsystem to use.
     * @param intake The IntakeSubsystem to use.
     */
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
        this(shooter, intake, indexer, 99); // A large number for effectively infinite shooting
    }

    /**
     * Called when the command is initially scheduled. Sets the initial state.
     */
    @Override
    public void initialize() {
        intake.run();
        state = SHOOT_STATES.Conveyor;
        timer.reset();
    }

    /**
     * Called repeatedly when this Command is scheduled to run. Executes the shooting state machine.
     */
    @Override
    public void execute() {

        switch (state) {
            case Conveyor:
                // MELHORIA: Detecta a peça na "câmara" via sensor de saída para reduzir latência de busca
                if (indexer.getExitSensor() || timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TO_SHOOT) {

                    // Se o subsistema estiver pronto (Bang-Bang estabilizado), atira sem passar por Acceleration
                    if (shooter.getShooterAtTarget()) {
                        state = SHOOT_STATES.Shooting;
                        intake.runTrigger();
                        timer.reset();
                    } else {
                        // Caso contrário, prepara para acelerar mantendo a peça pronta no gatilho
                        state = SHOOT_STATES.Acceleration;
                        intake.stopTrigger();
                    }
                }
                break;

            case Acceleration:
                // MELHORIA: Disparo instantâneo no microssegundo em que o RPM entra na janela estável
                if (shooter.getShooterAtTarget()) {
                    state = SHOOT_STATES.Shooting;
                    intake.runTrigger();

                    intake.run(); // Mantém pressão constante para a próxima peça já vir vindo

                    timer.reset();
                }
                break;

            case Shooting:
                // MELHORIA COMPETITIVA: Usa o sensor de saída como gatilho de interrupção (Borda de Descida).
                // Se o sensor ficar falso, a peça saiu fisicamente. Voltamos ao ciclo instantaneamente.
                boolean pieceHasLeft =!indexer.getExitSensor();

                if (pieceHasLeft || timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TRIGGERING) {
                    state = SHOOT_STATES.Conveyor;
                    intake.run();
                    timer.reset();
                    if (shooterCounter > 0) {
                        shooterCounter--;
                    }
                }
                break;
        }

        telemetryM.addData("Shoot State", state);
        telemetryM.addData("RPM Ready", shooter.getShooterAtTarget());
    }

    /**
     * Returns true when the command should end.
     * @return True if the desired number of shots has been completed.
     */
    @Override
    public boolean isFinished() {
        return shooterCounter <= 0;
    }

    /**
     * Called once the command ends or is interrupted. Stops all motors.
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){
        intake.stopTrigger();
        intake.stop();
        telemetryM.addData("Shoot State", "Finish");
    }
}