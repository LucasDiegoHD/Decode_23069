package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.templates.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterSubsystem;

import java.util.function.IntSupplier;

public class ShootCommandAutonomous extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final TelemetryManager telemetryM;
    private final IntSupplier shootCountSupplier;

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
    public ShootCommandAutonomous(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, IntSupplier shootCount) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.shooter = shooter;
        this.intake = intake;
        this.indexer = indexer;
        this.shootCountSupplier = shootCount;
        addRequirements(shooter, indexer);
    }

    public ShootCommandAutonomous(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, int shootCount) {
        this(shooter, intake, indexer, () -> shootCount);
    }

    public ShootCommandAutonomous(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
        this(shooter, intake, indexer, () -> 500);
    }

    @Override
    public void initialize() {
        this.shooterCounter = shootCountSupplier.getAsInt();
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
                    if (shooterCounter > 0) shooterCounter--;
                    state = SHOOT_STATES.FollowThrough;
                    timer.reset();
                }
                break;

            case FollowThrough:
                if (timer.milliseconds() > ShooterConstants.TRIGGER_FOLLOW_THROUGH_MS) {
                    intake.stopTrigger();
                    intake.stop();
                    if (shooterCounter > 0) {
                        cooldownTimer.reset();
                        state = SHOOT_STATES.Cooldown;
                    } else {
                        state = SHOOT_STATES.Conveyor;
                    }
                }
                break;

            case Cooldown:
                if (cooldownTimer.milliseconds() > ShooterConstants.DELAY_BETWEEN_SHOTS_MS) {
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
    public void end(boolean interrupted) {
        intake.stopTrigger();
        intake.stop();
        indexer.setShootingState(false);
    }
}