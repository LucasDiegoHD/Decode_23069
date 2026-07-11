package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;
    private final AllianceEnum alliance;

    private static final double MAX_ACCELERATION = 8.5;
    private static final double MAX_DECELERATION = 10.0;
    private static final double NOMINAL_VOLTAGE   = 13.5;
    private static final double MAX_VOLTAGE_SCALE = 1.25;

    // --- Constantes do Drive Straight ---
    private static final double TURN_DEADBAND = 0.04;
    private static final double HEADING_KD    = 0.1;
    private static final double MAX_COMP_PWR  = 0.3;

    private double currentMagnitude = 0.0;
    private double currentAngle = 0.0;
    private long   lastTime = 0;
    private double lastHeading = 0.0;

    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driverGamepad) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        this.alliance = DataStorage.alliance;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
        lastTime = System.currentTimeMillis();

        double rawY = driverGamepad.getLeftX();
        double rawX = driverGamepad.getLeftY();

        double targetX = rawX * Math.abs(rawX);
        double targetY = rawY * Math.abs(rawY);

        currentMagnitude = Math.hypot(targetX, targetY);
        currentAngle = (currentMagnitude > 0.01) ? Math.atan2(targetY, targetX) : 0.0;

        lastHeading = drivetrain.getFollower().getPose().getHeading();
    }

    @Override
    public void execute() {
        Pose p = drivetrain.getFollower().getPose();
        double heading = p.getHeading();

        double rawY = driverGamepad.getLeftX();
        double rawX = driverGamepad.getLeftY();
        double rawTurn = -driverGamepad.getRightX();

        double targetX = rawX * Math.abs(rawX);
        double targetY = rawY * Math.abs(rawY);
        double targetTurn = rawTurn * Math.abs(rawTurn);

        long currentTime = System.currentTimeMillis();
        double dt = Math.max((currentTime - lastTime) / 1000.0, 0.001);
        lastTime = currentTime;

        // --- Lógica de Translação (Intacta) ---
        double targetMagnitude = Math.hypot(targetX, targetY);
        double targetAngle = (targetMagnitude > 0.01)
                ? Math.atan2(targetY, targetX)
                : currentAngle;

        double delta;
        if (targetMagnitude >= currentMagnitude) {
            delta = MAX_ACCELERATION * dt;
        } else {
            delta = MAX_DECELERATION * dt;
        }

        if (Math.abs(targetMagnitude - currentMagnitude) <= delta) {
            currentMagnitude = targetMagnitude;
        } else {
            currentMagnitude += Math.copySign(delta, targetMagnitude - currentMagnitude);
        }

        if (targetMagnitude > 0.05) {
            double angleDiff = targetAngle - currentAngle;
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
            currentAngle += angleDiff * Math.min(1.0, currentMagnitude * 8.0 * dt);
        }

        double smoothX = currentMagnitude * Math.cos(currentAngle);
        double smoothY = currentMagnitude * Math.sin(currentAngle);

        double headingDelta = heading - lastHeading;
        while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
        while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;

        double angularVelocity = headingDelta / dt;
        lastHeading = heading;

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

        double voltage      = Math.max(drivetrain.getVoltage(), 10.0);
        double voltageScale = Math.min(NOMINAL_VOLTAGE / voltage, MAX_VOLTAGE_SCALE);

        drivetrain.getFollower().setTeleOpDrive(
                xField     * voltageScale,
                -yField    * voltageScale,
                finalTurnPower * voltageScale,
                true
        );
    }
}