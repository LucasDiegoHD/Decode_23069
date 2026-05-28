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

    private double currentMagnitude = 0.0;
    private double currentAngle = 0.0;
    private long lastTime = 0;

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
        currentMagnitude = 0.0;
        currentAngle = 0.0;
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
        double dt = Math.min((currentTime - lastTime) / 1000.0, 0.05);
        lastTime = currentTime;

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

        // Rotação field-centric
        double xField = smoothX * Math.cos(heading) - smoothY * Math.sin(heading);
        double yField = smoothX * Math.sin(heading) + smoothY * Math.cos(heading);

        if (alliance == AllianceEnum.Blue) {
            xField = -xField;
            yField = -yField;
        }

        drivetrain.getFollower().setTeleOpDrive(
                xField,
                -yField,
                targetTurn,
                true
        );
    }
}