package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * A command for controlling the robot's drivetrain during the tele-operated period.
 * It uses field-centric control, meaning the robot's movement is relative to the field,
 * not the robot's orientation.
 */
public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;
    private final AllianceEnum alliance;

    // --- CONTROLE DE TRAÇÃO (Slew Rate Limiter) ---
    private static final double ACCELERATION_TIME_SECONDS = 0.25;

    // Variáveis de memória para a rampa
    private double currentY = 0.0;
    private double currentX = 0.0;
    private double currentTurn = 0.0;
    private long lastTime = 0;

    /**
     * Creates a new TeleOpDriveCommand.
     *
     * @param drivetrain    The DrivetrainSubsystem to control.
     * @param driverGamepad The gamepad used for driving.
     */
    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driverGamepad) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        this.alliance = DataStorage.alliance;
        addRequirements(drivetrain);
    }

    /**
     * Called when the command is initially scheduled. Prepares the follower for teleop mode.
     */
    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
        lastTime = System.currentTimeMillis(); // Inicia o relógio do acelerador
    }

    /**
     * Called repeatedly while the command is scheduled. Reads joystick inputs,
     * calculates field-centric drive vectors, and commands the drivetrain.
     */
    @Override
    public void execute() {
        Pose p = drivetrain.getFollower().getPose();
        double heading = p.getHeading();

        double rawY = driverGamepad.getLeftX(); // Forward/backward
        double rawX = driverGamepad.getLeftY(); // Strafe left/right
        double rawTurn = -driverGamepad.getRightX();

        // Se o piloto mexer só um pouquinho (0.2), vira 0.008 (super devagar).
        // Se empurrar tudo (1.0), vira 1.0 (força total).
        double targetY = Math.pow(rawY, 3);
        double targetX = Math.pow(rawX, 3);
        double targetTurn = rawTurn;

        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        currentY = applySlewRate(targetY, currentY, dt);
        currentX = applySlewRate(targetX, currentX, dt);

        double xField = currentX * Math.cos(heading) - currentY * Math.sin(heading);
        double yField = currentX * Math.sin(heading) + currentY * Math.cos(heading);

        if(alliance == AllianceEnum.Blue){
            xField = -xField;
            yField = -yField;
        }

        drivetrain.getFollower().setTeleOpDrive(
                xField, // Forward/backward power
                -yField, // Strafe power
                targetTurn, // Turn power
                true
        );
    }

    /**
     * Método auxiliar que calcula a rampa de aceleração limitando o salto brusco de energia.
     */
    private double applySlewRate(double target, double current, double dt) {
        double maxChange = (1.0 / ACCELERATION_TIME_SECONDS) * dt;
        double error = target - current;

        if (Math.abs(error) > maxChange) {
            return current + Math.copySign(maxChange, error);
        } else {
            return target;
        }
    }
}