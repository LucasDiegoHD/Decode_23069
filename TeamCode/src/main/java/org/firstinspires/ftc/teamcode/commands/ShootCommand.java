package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final GamepadEx driver;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();
    private enum SHOOT_STATES {
        Conveyor,
        Acceleration,
        Shooting,
        FollowThrough,
        Cooldown
    }

    private SHOOT_STATES state;
    private int shooterCounter;
    private final TelemetryManager telemetryM;
    private final IndexerSubsystem indexer;

    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, int shoots, GamepadEx driver) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.indexer = indexer;
        this.shooterCounter = shoots;
        this.shooter = shooter;
        this.intake = intake;
        this.driver = driver;
        addRequirements(shooter, indexer);
    }

    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, GamepadEx driver) {
        this(shooter, intake, indexer, 500, driver);
    }

    @Override
    public void initialize() {
        intake.run();
        intake.stopTrigger();
        state = SHOOT_STATES.Conveyor;
        timer.reset();
        indexer.setShootingState(true);
        if (driver != null && driver.gamepad != null) {
            driver.gamepad.rumble(150);
        }
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
                    if (shooter.getShooterAtTarget()) {
                        state = SHOOT_STATES.Shooting;
                        shooter.anticipateShot();
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

                if (shooter.getShooterAtTarget()) {
                    state = SHOOT_STATES.Shooting;
                    shooter.anticipateShot();
                    intake.runTrigger();
                    intake.run();

                    timer.reset();
                }
                break;

            case Shooting:
                boolean pieceHasLeft = !indexer.getExitSensor();

                if (pieceHasLeft || timer.milliseconds() > ShooterConstants.TRIGGER_TIMER_TRIGGERING) {
                    if (shooterCounter > 0) {
                        shooterCounter--;
                    }

                    state = SHOOT_STATES.FollowThrough;
                    timer.reset();
                }
                break;

            case FollowThrough:

                if (timer.milliseconds() > ShooterConstants.TRIGGER_FOLLOW_THROUGH_MS) {

                    if (shooterCounter > 0) {
                        state = SHOOT_STATES.Cooldown;

                        // DESLIGA AQUI (Apenas 1 vez!)
                        intake.stopTrigger();
                        intake.stop();
                        cooldownTimer.reset();
                    } else {
                        state = SHOOT_STATES.Conveyor;
                        intake.stopTrigger();
                        intake.stop();
                    }
                }
                break;

            case Cooldown:
                double time = cooldownTimer.milliseconds();

                if (time > ShooterConstants.DELAY_BETWEEN_SHOTS_MS) {
                    state = SHOOT_STATES.Conveyor;

                    if (indexer.getExitSensor()) intake.stop();
                    timer.reset();
                }
                break;
        }

        telemetryM.addData("Shoot State", state);
        telemetryM.addData("Shots Left", shooterCounter);
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
    }
}