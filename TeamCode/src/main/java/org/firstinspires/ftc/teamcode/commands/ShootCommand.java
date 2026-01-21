package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.LedCommands;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

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
    private final LEDSubsystem ledSubsystem;
    private final boolean lastSensor = false;

    /**
     * Constructs a new ShootCommand.
     * @param shooter The ShooterSubsystem to use.
     * @param intake The IntakeSubsystem to use.
     * @param shoots The number of pieces to shoot. Use a high number for continuous shooting.
     */
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, int shoots, LEDSubsystem led) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.indexer = indexer;
        this.shooterCounter = shoots;
        this.shooter = shooter;
        this.intake = intake;
        this.ledSubsystem = led;
        addRequirements(shooter, indexer);
    }

    /**
     * Constructs a new ShootCommand for continuous shooting.
     * @param shooter The ShooterSubsystem to use.
     * @param intake The IntakeSubsystem to use.
     */
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, LEDSubsystem led) {
        this(shooter, intake, indexer, 99, led); // A large number for effectively infinite shooting
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
                if (indexer.getExitSensor() || timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TO_SHOOT) {

                    if (shooter.getShooterAtTarget()) {
                        state = SHOOT_STATES.Shooting;
                        shooter.anticipateShot();
                        intake.runTrigger();
                        ledSubsystem.setPattern(LEDSubsystem.GREEN);
                        timer.reset();
                    } else {
                        state = SHOOT_STATES.Acceleration;
                        ledSubsystem.setPattern(LEDSubsystem.OFF);
                        intake.stopTrigger();
                    }
                }
                break;

            case Acceleration:
                if (shooter.getShooterAtTarget()) {
                    state = SHOOT_STATES.Shooting;
                    shooter.anticipateShot();
                    intake.runTrigger();
                    ledSubsystem.setPattern(LEDSubsystem.GREEN);
                    intake.run();

                    timer.reset();
                }
                break;

            case Shooting:

                boolean pieceHasLeft =!indexer.getExitSensor();

                if (pieceHasLeft || timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TRIGGERING) {
                    state = SHOOT_STATES.Conveyor;
                    intake.run();
                    ledSubsystem.setPattern(LEDSubsystem.OFF);
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