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

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();

    private static final double TRIGGER_CLEAR_DELAY_MS = 100;

    private enum SHOOT_STATES {
        Conveyor,
        Acceleration,
        Shooting,
        Cooldown
    }

    private SHOOT_STATES state;
    private int shooterCounter;
    private final TelemetryManager telemetryM;
    private final IndexerSubsystem indexer;
    private final LEDSubsystem ledSubsystem;

    public ShootCommandAutonomous(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, int shoots, LEDSubsystem led) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.indexer = indexer;
        this.shooterCounter = shoots;
        this.shooter = shooter;
        this.intake = intake;
        this.ledSubsystem = led;
        addRequirements(shooter, indexer);
    }

    public ShootCommandAutonomous(ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, LEDSubsystem led) {
        this(shooter, intake, indexer, 99, led);
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

                    if (shooter.isReady()) {
                        state = SHOOT_STATES.Shooting;
                        shooter.anticipateShot();
                        intake.runTrigger();
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
                    intake.runTrigger();
                    timer.reset();
                }
                break;

            case Shooting:
                boolean pieceHasLeft = !indexer.getExitSensor();

                if (pieceHasLeft || timer.milliseconds() > 1200) {

                    if (shooterCounter > 0) {
                        shooterCounter--;
                    }

                    if (shooterCounter > 0) {
                        state = SHOOT_STATES.Cooldown;
                        intake.stopTrigger();
                        intake.stop();
                        cooldownTimer.reset();
                    } else {
                        state = SHOOT_STATES.Conveyor;
                    }
                }
                break;

            case Cooldown:
                double time = cooldownTimer.milliseconds();

                if (time > 300) {
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
        telemetryM.addData("Shoot State", "Finish");
    }
}